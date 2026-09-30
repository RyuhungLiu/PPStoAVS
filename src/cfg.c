#include "cfg.h"
#include "flash_io.h"
#include <stddef.h>
#include <string.h>

#define CFG_MAGIC           0x34464350u     /* 'PCF4'：cfg_t 32 字节（v0.9.0 起） */
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
    uint32_t crc;           /* magic ~ cfg */
    uint32_t pad[21];
} cfg_page_t;

_Static_assert(sizeof(cfg_t) == 32, "cfg_t size");
_Static_assert(sizeof(cfg_page_t) == FLASH_PAGE_SIZE, "cfg_page_t size");

static cfg_t cur;
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
    return p->magic == CFG_MAGIC && p->crc == crc32_calc(p, offsetof(cfg_page_t, crc)) && cfg_valid(&p->cfg);
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
    return page_old_load(page, CFG_MAGIC_V3, CFG_V3_SIZE, out) || page_old_load(page, CFG_MAGIC_V2, CFG_V2_SIZE, out) ||
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
    cur_seq = 0;
    cur_slot = 1;
    for (uint8_t s = 0; s < CFG_FLASH_PAGES; s++)
    {
        const cfg_page_t *p = slot_page(s);
        cfg_t v1;
        if (page_valid(p) && p->seq >= cur_seq)
        {
            cur = p->cfg;
            cur_seq = p->seq;
            cur_slot = s;
        }
        else if (page_legacy_load(p, &v1) && p->seq >= cur_seq)
        {
            cur = v1;
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
    cur.reserved3 = 0;
    save_state = SAVE_ERASE;    /* 保存中途再次修改：从擦除重新开始 */
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
