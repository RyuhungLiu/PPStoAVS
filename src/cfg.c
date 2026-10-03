#include "cfg.h"
#include "flash_io.h"
#include "pd_defs.h"
#include <stddef.h>
#include <string.h>

#define CFG_MAGIC           0x35464350u     /* 'PCF5'：cfg_t 32 字节 + cfg_ext_t（v0.12.0 起） */
#define CFG_MAGIC_V4        0x34464350u     /* 'PCF4'：cfg_t 32 字节（v0.9.0 ~ v0.11.x） */
#define CFG_V4_SIZE         32
#define CFG_MAGIC_V3        0x33464350u     /* 'PCF3'：cfg_t 前 24 字节（v0.7.0 ~ v0.8.0） */
#define CFG_V3_SIZE         24
#define CFG_MAGIC_V2        0x32464350u     /* 'PCF2'：cfg_t 前 20 字节（v0.6.x） */
#define CFG_V2_SIZE         20
#define CFG_MAGIC_V1        0x47464350u     /* 'PCFG'：cfg_t 前 16 字节（v0.2.0 ~ v0.5.0） */
#define CFG_V1_SIZE         16

typedef struct
{
    uint32_t magic;
    uint32_t seq;
    cfg_t    cfg;
    cfg_ext_t ext;
    uint32_t crc;           /* magic ~ ext */
    uint32_t pad[3];
} cfg_page_t;

_Static_assert(sizeof(cfg_t) == 32, "cfg_t size");
_Static_assert(sizeof(cfg_ext_t) == 72, "cfg_ext_t size");
_Static_assert(sizeof(cfg_page_t) == FLASH_PAGE_SIZE, "cfg_page_t size");

static cfg_t cur;
static cfg_ext_t cur_x;
static uint32_t cur_seq;
static uint8_t cur_slot;            /* 最近一次有效数据所在页 */

/* 保存：先擦目标页，下一个空闲窗口再编程 */
static enum { SAVE_IDLE, SAVE_ERASE, SAVE_PROGRAM } save_state;
static cfg_page_t save_buf __attribute__((aligned(4)));

static const cfg_page_t *slot_page(uint8_t slot)
{
    return (const cfg_page_t *)(CFG_FLASH_ADDR + slot * FLASH_PAGE_SIZE);
}

static bool page_valid(const cfg_page_t *p)
{
    return p->magic == CFG_MAGIC && p->crc == crc32_calc(p, offsetof(cfg_page_t, crc)) && cfg_valid(&p->cfg) &&
           cfg_ext_valid(&p->ext);
}

/* 旧格式（magic、seq、cfg 前 size 字节、crc）：已有字段照旧，新增字段取默认值 */
static bool page_old_load(const cfg_page_t *page, uint32_t magic, uint8_t size, cfg_t *out)
{
    const uint8_t *b = (const uint8_t *)page;
    uint32_t crc;
    memcpy(&crc, b + 8 + size, 4);
    if (page->magic != magic || crc != crc32_calc(b, 8 + size))
        return false;
    cfg_defaults(out);
    memcpy(out, b + 8, size);
    if (!(out->flags2 & CFG2_ID_PT))
        out->flags2 |= CFG2_ID_BE;      /* 后端自订身份默认开启；旧设置里选了身份透传的保持不变 */
    return cfg_valid(out);
}

static bool page_legacy_load(const cfg_page_t *page, cfg_t *out)
{
    return page_old_load(page, CFG_MAGIC_V4, CFG_V4_SIZE, out) || page_old_load(page, CFG_MAGIC_V3, CFG_V3_SIZE, out) || page_old_load(page, CFG_MAGIC_V2, CFG_V2_SIZE, out) ||
           page_old_load(page, CFG_MAGIC_V1, CFG_V1_SIZE, out);
}

void cfg_defaults(cfg_t *c)
{
    memset(c, 0, sizeof(*c));
    c->mode = CFG_MODE_AVS;
    c->hide_fixed = 0;
    c->max_mv = 20000;
    c->max_ma = 3000;
    c->ovp_pct = 5;
    c->uvp_pct = 5;
    c->ocp_ma = 3500;
    c->ocp_ms = 50;
    c->comp_mode = CFG_COMP_R;
    c->comp_val = 15;       /* 15mΩ：背靠背 MOS + 5mΩ 采样电阻 + 走线 */
    c->pps_min_dv = 33;     /* 自订 PPS 建议值：3.3~16V 3A（未启用） */
    c->pps_max_dv = 160;
    c->pps_ma50 = 60;
    c->flags2 = CFG2_ID_BE; /* 默认：后端以自订身份应答设备 */
    c->fe_vid = 0x5A1E;     /* 自编的 VID/PID，并非任何厂商注册的 ID */
    c->fe_pid = 0x30A6;
    c->be_vid = 0x5A1E;
    c->be_pid = 0x30A5;
}

void cfg_ext_defaults(cfg_ext_t *x)
{
    memset(x, 0, sizeof(*x));
    /* 通用数据：无源 C 口线材（<10ns、3A、20V、USB 2.0、不支持 EPR、不需要 VCONN） */
    x->cable[0] = (3u << 27) | 0x1209u;                         /* 无源线材，VID 同 PI_VID */
    x->cable[2] = 0x0001u << 16;                                /* PID（同 PI_PID） */
    x->cable[3] = (2u << 18) | (1u << 13) | (1u << 5);
    /* 前端：无源 5A 线材（C-C、EPR、<10ns、50V、5A、USB 2.0） */
    x->fcable[0] = (3u << 27) | 0x1209u;
    x->fcable[2] = 0x0001u << 16;
    x->fcable[3] = (2u << 18) | (1u << 17) | (1u << 13) | (3u << 9) | (2u << 5);
}

bool cfg_ext_fe_5a(const cfg_ext_t *x)
{
    return ((x->fcable[3] >> 5) & 3) == 2;
}

bool cfg_ext_cable_5a(const cfg_ext_t *x)
{
    return ((x->cable[3] >> 5) & 3) == 2;
}

/* 自订 PDO 的合法范围：Fixed 5~20V（50mV 倍数）、PPS 3.3~21V、电流 0.5~5A；5V Fixed 必须有；AVS 至多 1 个，须有 15V Fixed（AVS 到 20V 另需 20V Fixed，没有则 AVS 只到 15V） */
bool cfg_ext_valid(const cfg_ext_t *x)
{
    if ((x->flags & ~CFGX_MASK) || x->pdo_n > CFGX_MAX_PDOS)
        return false;
    if (!(x->flags & CFGX_PDO))
        return true;
    uint8_t avs = 0, v5 = 0, v15 = 0, v20 = 0;
    for (uint8_t i = 0; i < x->pdo_n; i++)
    {
        pdo_t p = pd_parse_pdo(x->pdo[i]);
        if (p.type == FPDO)
        {
            if (p.max_mv < 5000 || p.max_mv > 20000 || p.max_mv % 100 || p.max_ma < 500 || p.max_ma > CFG_MAX_MA_5A ||
                p.max_ma % 50 || (x->pdo[i] & 0xC0000000u))
                return false;
            v5 += p.max_mv == 5000;
            v15 += p.max_mv == 15000;
            v20 += p.max_mv == 20000;
        }
        else if (p.type == PPS_PDO)
        {
            if (p.min_mv < 3300 || p.max_mv > 21000 || p.min_mv >= p.max_mv || p.max_ma < 500 || p.max_ma > CFG_MAX_MA_5A)
                return false;
        }
        else if (p.type == SPR_AVS_PDO)
            avs++;
        else
            return false;
        for (uint8_t j = 0; j < i; j++)
        {
            if (x->pdo[j] == x->pdo[i])
                return false;
        }
    }
    return v5 == 1 && avs <= 1 && (!avs || v15 == 1);
}

bool cfg_valid(const cfg_t *c)
{
    bool a5 = (c->flags & CFG_FE_EMARKER) != 0;
    if ((c->flags & CFG_EPR_AVS) && !a5)
        return false;   /* EPR 需要虚拟 E-Marker */
    if ((c->flags & CFG_FIX_CUSTOM) &&
        ((c->flags & CFG_FIX12) || c->fix_dv < CFG_FIX_DV_MIN || c->fix_dv > CFG_FIX_DV_MAX || c->fix_ma50 < 10 ||
         c->fix_ma50 > 100))
        return false;   /* 自订 Fixed 与 12V 转换互斥 */
    if ((c->flags2 & CFG2_PPS_CUSTOM) &&
        (c->pps_min_dv < CFG_PPS_DV_MIN || c->pps_max_dv > CFG_PPS_DV_MAX || c->pps_max_dv <= c->pps_min_dv ||
         c->pps_ma50 < 10 || c->pps_ma50 > 100))
        return false;
    if ((c->flags2 & CFG2_ID_PT) && (c->flags2 & (CFG2_ID_FE | CFG2_ID_BE)))
        return false;   /* 自订身份与身份透传互斥 */
    if (((c->flags2 & CFG2_ID_FE) && !c->fe_vid) || ((c->flags2 & CFG2_ID_BE) && !c->be_vid))
        return false;
    if (c->comp_mode > CFG_COMP_R || (c->comp_mode == CFG_COMP_V && c->comp_val > CFG_COMP_V_MAX) ||
        (c->comp_mode == CFG_COMP_R && c->comp_val > CFG_COMP_R_MAX))
        return false;
    return c->mode <= CFG_MODE_FIXED && (c->hide_fixed & ~0x0Fu) == 0 && (c->flags & ~CFG_FLAGS_MASK) == 0 &&
           (c->max_mv == 15000 || c->max_mv == 20000) &&
           c->max_ma >= 500 && c->max_ma <= (a5 ? CFG_MAX_MA_5A : CFG_MAX_MA) && c->max_ma % 50 == 0 &&
           c->ovp_pct >= 1 && c->ovp_pct <= 20 && c->uvp_pct >= 1 && c->uvp_pct <= 20 &&
           c->ocp_ma >= 500 && c->ocp_ma <= (a5 ? CFG_OCP_MAX_MA_5A : CFG_OCP_MAX_MA) && c->ocp_ms >= 1 &&
           c->ocp_ms <= 1000;
}

void cfg_init(void)
{
    cfg_defaults(&cur);
    cfg_ext_defaults(&cur_x);
    cur_seq = 0;
    cur_slot = 1;
    for (uint8_t s = 0; s < CFG_FLASH_PAGES; s++)
    {
        const cfg_page_t *p = slot_page(s);
        cfg_t v1;
        if (page_valid(p) && p->seq >= cur_seq)
        {
            cur = p->cfg;
            cur_x = p->ext;
            cur_seq = p->seq;
            cur_slot = s;
        }
        else if (page_legacy_load(p, &v1) && p->seq >= cur_seq)
        {
            cur = v1;
            cfg_ext_defaults(&cur_x);
            if (cur.flags3 & (CFG3_OLD_NO5A | CFG3_OLD_5A))
            {
                /* v0.11.x 的线材策略 3A / 5A → 虚拟 E-Marker */
                cur_x.flags = CFGX_CABLE;
                cur_x.cable[3] = (cur_x.cable[3] & ~(3u << 5)) | ((cur.flags3 & CFG3_OLD_5A) ? 2u << 5 : 1u << 5);
            }
            cur.flags3 &= CFG3_MASK;
            cur_seq = p->seq;
            cur_slot = s;
        }
    }
}

const cfg_t *cfg(void)
{
    return &cur;
}

bool cfg_set(const cfg_t *c)
{
    if (!cfg_valid(c))
        return false;
    cur = *c;
    cur.flags2 &= CFG2_MASK;
    cur.reserved2 = 0;
    cur.flags3 &= CFG3_MASK;
    save_state = SAVE_ERASE;    /* 保存中途再次修改：从擦除重新开始 */
    return true;
}

const cfg_ext_t *cfg_ext(void)
{
    return &cur_x;
}

bool cfg_ext_set(const cfg_ext_t *x)
{
    if (!cfg_ext_valid(x))
        return false;
    cur_x = *x;
    cur_x.reserved = 0;
    save_state = SAVE_ERASE;
    return true;
}

bool cfg_save_pending(void)
{
    return save_state != SAVE_IDLE;
}

bool cfg_flush_step(void)
{
    uint8_t slot = cur_slot ^ 1;    /* 写另一页，掉电时旧页仍有效 */
    switch (save_state)
    {
    case SAVE_ERASE:
        flash_page_erase(CFG_FLASH_ADDR + slot * FLASH_PAGE_SIZE);
        save_state = SAVE_PROGRAM;
        return true;

    case SAVE_PROGRAM:
        memset(&save_buf, 0, sizeof(save_buf));
        save_buf.magic = CFG_MAGIC;
        save_buf.seq = cur_seq + 1;
        save_buf.cfg = cur;
        save_buf.ext = cur_x;
        save_buf.crc = crc32_calc(&save_buf, offsetof(cfg_page_t, crc));
        if (flash_page_program(CFG_FLASH_ADDR + slot * FLASH_PAGE_SIZE, (const uint32_t *)&save_buf))
        {
            cur_seq = save_buf.seq;
            cur_slot = slot;
        }
        save_state = SAVE_IDLE;
        return true;

    default:
        return false;
    }
}
