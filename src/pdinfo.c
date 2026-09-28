#include "pdinfo.h"
#include "board.h"
#include "bridge.h"
#include "cfg.h"
#include "evlog.h"
#include "pd_defs.h"
#include "timebase.h"

#define T_BSTAT_FRESH_MS        1000    /* 充电器读电池状态时，缓存超过这个时间就让后端重读 */
#define T_BSTAT_ALERT_MS        6000    /* 充电器没在读电量时，每 6s 重读并发 Alert（与 iPhone 相同） */
#define ADO_FIXED_BATTERY0      (1u << 20)

/* SKEDB / SCEDB / BCDB / MIDB 共同的开头：VID u16、PID u16 */
#define EXT_VID                 0
#define EXT_PID                 2
#define SCEDB_MIN_LEN           24      /* PD3.0；PD3.1 多 1 字节 EPR Source PDP */
#define SCEDB_SPR_PDP           23
#define SCEDB_EPR_PDP           24
#define SKEDB_SPR_PDP_MIN       18
#define SKEDB_EPR_PDP_MIN       21
#define BCDB_LEN                9
#define BCDB_INVALID_REF        0x01    /* Battery Type bit0 */
#define BSDO_INVALID_REF        (1u << 8)
#define SSDB_LEN                7
#define MIDB_MIN_LEN            4
#define ADO_BATTERY_CHANGE      (1u << 25)  /* Type of Alert bit1 */
#define ADO_BATTERY_MASK        0x00FF0000u /* 固定/热插拔电池位 */

/* 设备 */
static uint8_t dev_q;
static bool dev_started;
static bool dev_attached;
static pi_ident_t dev_id;
static bool dev_skedb_ok, dev_bcap_ok, dev_bsdo_ok;
static uint8_t dev_skedb[SKEDB_LEN];
static uint8_t dev_bcap[BCDB_LEN];
static uint32_t dev_bsdo;
static uint32_t dev_bsdo_ts;
static uint32_t alert_wait;             /* 设备 Alert：等电池状态重读完再转发 */
static uint32_t alert_fwd;

/* 充电器 */
static uint8_t chg_q;
static pi_ident_t chg_id;
static uint8_t chg_scedb_len, chg_status_len, chg_midb_len;
static bool chg_sido_ok;
static uint8_t chg_scedb[PI_EXT_MAX];
static uint8_t chg_status[SSDB_LEN];
static uint8_t chg_midb[PI_EXT_MAX];
static uint32_t chg_sido;

bool pdinfo_on(void)
{
    return !(cfg()->flags2 & CFG2_NO_INFO);
}

bool pdinfo_id_on(void)
{
    return pdinfo_on() && (cfg()->flags2 & CFG2_ID_PT);
}

static uint16_t get16(const uint8_t *p)
{
    return p[0] | (p[1] << 8);
}

static void put16(uint8_t *p, uint16_t v)
{
    p[0] = v;
    p[1] = v >> 8;
}

/* 身份透传关闭：VID/PID 换成本机，XID、版本清零 */
static void own_vid_pid(uint8_t *d, bool with_xid)
{
    put16(&d[EXT_VID], PI_VID);
    put16(&d[EXT_PID], PI_PID);
    if (with_xid)
        memset(&d[4], 0, 6);    /* XID u32、FW/HW 版本 */
}

/* 后端实际能提供的最大功率（W） */
static uint8_t back_pdp(void)
{
    uint8_t blob[PD_MAX_DATA_OBJS * 5];
    uint8_t n = bridge_back_caps_peek(blob);
    uint32_t mw = 0;
    for (uint8_t i = 0; i < n; i++)
    {
        pdo_t p = pd_parse_pdo(pd_get_u32(&blob[i * 4]));
        uint32_t w = (uint32_t)p.max_mv * p.max_ma;
        if (p.type == SPR_AVS_PDO)
        {
            uint32_t w20 = 20000u * p.max_ma_20v;
            w = 15000u * p.max_ma;
            if (w20 > w)
                w = w20;
        }
        if (w > mw)
            mw = w;
    }
    return mw / 1000000;
}

static void log_ident(uint8_t side, const pi_ident_t *id)
{
    ev_ident_t e = {side, (uint16_t)id->vdo[0], id->n >= 3 ? (uint16_t)(id->vdo[2] >> 16) : 0,
                    id->n >= 3 ? (uint16_t)id->vdo[2] : 0};
    evlog_add(EV_IDENT, &e, sizeof(e));
}

static void set_ident(pi_ident_t *dst, const uint32_t *vdo, uint8_t n)
{
    if (n > PI_MAX_VDOS)
        n = PI_MAX_VDOS;
    dst->n = n;
    memcpy(dst->vdo, vdo, n * 4);
}

/* ================= 设备 ================= */

void pdinfo_dev_reset(void)
{
    dev_q = 0;
    dev_started = false;
    dev_id.n = 0;
    dev_skedb_ok = dev_bcap_ok = dev_bsdo_ok = false;
    alert_wait = alert_fwd = 0;
}

void pdinfo_dev_attached(bool on)
{
    dev_attached = on;
}

void pdinfo_dev_start(bool pd3)
{
    if (dev_started)
        return;
    dev_started = true;
    if (pd3 && pdinfo_on())
    {
        dev_q = PI_Q_IDENT | PI_Q_EXTCAPS | PI_Q_BCAP | PI_Q_BSTAT;
        alert_wait = ADO_BATTERY_CHANGE | ADO_FIXED_BATTERY0;   /* 读到电量后通知充电器 */
    }
}

uint8_t pdinfo_dev_next_query(void)
{
    if (!pdinfo_on())
        dev_q = 0;
    /*
     * 部分充电器只在 Fixed 合约下主动查询 Sink_Capabilities_Extended，PPS 合约（强制 PPS）下要靠 Alert 才开始读电量：
     * 充电器一段时间没读电量时，后端重读并主动发 Alert（充电器在读时缓存会被刷新，不会触发）
     */
    else if (dev_started && dev_bsdo_ok && !(dev_q & PI_Q_BSTAT) && millis() - dev_bsdo_ts > T_BSTAT_ALERT_MS)
    {
        dev_q |= PI_Q_BSTAT;
        if (!alert_wait)
            alert_wait = ADO_BATTERY_CHANGE | ADO_FIXED_BATTERY0;
    }
    return dev_q & -dev_q;      /* 最低位 */
}

void pdinfo_dev_query_done(uint8_t q)
{
    dev_q &= ~q;
    if ((q & PI_Q_BSTAT) && alert_wait)
    {
        if (dev_bsdo_ok)
            alert_fwd = alert_wait;     /* 没有电池（设备不支持）就不通知 */
        alert_wait = 0;
    }
}

void pdinfo_dev_ident(const uint32_t *vdo, uint8_t n)
{
    set_ident(&dev_id, vdo, n);
    if (n)
        log_ident(1, &dev_id);
}

void pdinfo_dev_skedb(const uint8_t *d, uint8_t len)
{
    if (len < SKEDB_LEN)
        return;
    memcpy(dev_skedb, d, SKEDB_LEN);
    dev_skedb_ok = true;
}

void pdinfo_dev_bcap(const uint8_t *d, uint8_t len)
{
    if (len < BCDB_LEN || (d[8] & BCDB_INVALID_REF))
        return;
    memcpy(dev_bcap, d, BCDB_LEN);
    dev_bcap_ok = true;
}

void pdinfo_dev_bsdo(uint32_t bsdo)
{
    if (bsdo & BSDO_INVALID_REF)
        return;
    dev_bsdo = bsdo;
    dev_bsdo_ok = true;
    dev_bsdo_ts = millis();
}

void pdinfo_dev_alert(uint32_t ado)
{
    if (!pdinfo_on() || !(ado & ADO_BATTERY_CHANGE))
        return;
    dev_q |= PI_Q_BSTAT;
    alert_wait = ADO_BATTERY_CHANGE | (ado & ADO_BATTERY_MASK);
}

/* ---- 前端应答充电器 ---- */

bool pdinfo_fe_skedb(uint8_t *out)
{
    if (!pdinfo_on() || !dev_skedb_ok)
        return false;
    const cfg_t *c = cfg();
    uint8_t pdp = (uint32_t)c->max_mv * c->max_ma / 1000000;
    memcpy(out, dev_skedb, SKEDB_LEN);
    if (!pdinfo_id_on())
        own_vid_pid(out, true);
    /* Sink Modes 描述的是本机前端：支持 PPS；AVS 仅在 Lab 二次握手打开时声明 */
    uint8_t m = (out[SKEDB_SINK_MODES] & ~SINK_MODE_AVS) | SINK_MODE_PPS;
    if (c->flags & CFG_FE_AVS_2ND)
        m |= SINK_MODE_AVS;
    out[SKEDB_SINK_MODES] = m;
    for (uint8_t i = SKEDB_SPR_PDP_MIN; i < SKEDB_SPR_PDP_MIN + 3; i++)
    {
        if (out[i] > pdp)
            out[i] = pdp;
    }
    memset(&out[SKEDB_EPR_PDP_MIN], 0, 3);     /* 本机不请求 20V 以上 */
    return true;
}

bool pdinfo_fe_ident(pi_ident_t *out)
{
    if (!pdinfo_id_on() || dev_id.n == 0)
        return false;
    *out = dev_id;
    return true;
}

bool pdinfo_fe_ident_pending(void)
{
    /* 设备已插入但还没读到身份（合约未建立或查询排队中）：充电器常在前端新合约后立刻查询，与后端查询赛跑 */
    return pdinfo_id_on() && dev_attached && (!dev_started || (dev_q & PI_Q_IDENT));
}

void pdinfo_fe_bcap(uint8_t ref, uint8_t *out)
{
    memset(out, 0, BCDB_LEN);
    if (ref != 0 || !dev_bcap_ok)
    {
        out[8] = BCDB_INVALID_REF;
        return;
    }
    memcpy(out, dev_bcap, BCDB_LEN);
    if (!pdinfo_id_on())
        own_vid_pid(out, false);
}

uint32_t pdinfo_fe_bsdo(uint8_t ref)
{
    if (ref != 0 || !dev_bsdo_ok)
        return BSDO_INVALID_REF;
    if (dev_started && millis() - dev_bsdo_ts > T_BSTAT_FRESH_MS)
        dev_q |= PI_Q_BSTAT;    /* 下次读到的是新数据 */
    return dev_bsdo;
}

uint32_t pdinfo_fe_take_alert(void)
{
    uint32_t a = alert_fwd;
    alert_fwd = 0;
    return pdinfo_on() ? a : 0;
}

/* ================= 充电器 ================= */

static bool chg_started;

void pdinfo_chg_reset(void)
{
    chg_q = 0;
    chg_started = false;
    chg_id.n = 0;
    chg_scedb_len = chg_status_len = chg_midb_len = 0;
    chg_sido_ok = false;
}

void pdinfo_chg_start(bool pd3)
{
    if (chg_started)
        return;
    chg_started = true;
    if (pd3 && pdinfo_on())
        chg_q = PI_Q_EXTCAPS | PI_Q_SIDO | PI_Q_STATUS | PI_Q_IDENT | PI_Q_MIDB;
}

uint8_t pdinfo_chg_next_query(void)
{
    if (!pdinfo_on())
        chg_q = 0;
    return chg_q & -chg_q;
}

void pdinfo_chg_query_done(uint8_t q)
{
    chg_q &= ~q;
}

void pdinfo_chg_ident(const uint32_t *vdo, uint8_t n)
{
    set_ident(&chg_id, vdo, n);
    if (n)
        log_ident(0, &chg_id);
}

void pdinfo_chg_scedb(const uint8_t *d, uint8_t len)
{
    if (len < SCEDB_MIN_LEN)
        return;
    chg_scedb_len = len > PI_EXT_MAX ? PI_EXT_MAX : len;
    memcpy(chg_scedb, d, chg_scedb_len);
}

void pdinfo_chg_sido(uint32_t sido)
{
    chg_sido = sido;
    chg_sido_ok = true;
}

void pdinfo_chg_status(const uint8_t *d, uint8_t len)
{
    chg_status_len = len > SSDB_LEN ? SSDB_LEN : len;
    memcpy(chg_status, d, chg_status_len);
}

void pdinfo_chg_midb(const uint8_t *d, uint8_t len)
{
    if (len < MIDB_MIN_LEN)
        return;
    chg_midb_len = len > PI_EXT_MAX ? PI_EXT_MAX : len;
    memcpy(chg_midb, d, chg_midb_len);
}

/* ---- 后端应答设备 ---- */

uint8_t pdinfo_be_scedb(uint8_t *out)
{
    if (!pdinfo_on() || chg_scedb_len == 0)
        return 0;
    memcpy(out, chg_scedb, chg_scedb_len);
    if (!pdinfo_id_on())
        own_vid_pid(out, true);
    out[SCEDB_SPR_PDP] = back_pdp();
    if (chg_scedb_len > SCEDB_EPR_PDP)
        out[SCEDB_EPR_PDP] = 0;     /* 后端只有 SPR */
    return chg_scedb_len;
}

/* Source_Info：端口类型沿用充电器（没有则按“保证”），三个 PDP 都按后端能力 */
bool pdinfo_be_sido(uint32_t *out)
{
    if (!pdinfo_on())
        return false;
    uint32_t pdp = back_pdp();
    uint32_t type = chg_sido_ok ? (chg_sido & (1u << 31)) : (1u << 31);
    *out = type | (pdp << 16) | (pdp << 8) | pdp;
    return true;
}

uint8_t pdinfo_be_status(uint8_t *out)
{
    if (!pdinfo_on() || chg_status_len == 0)
        return 0;
    memcpy(out, chg_status, chg_status_len);
    if (chg_started)
        chg_q |= PI_Q_STATUS;       /* 下次回答新数据 */
    return chg_status_len;
}

uint8_t pdinfo_be_midb(uint8_t *out)
{
    if (!pdinfo_id_on() || chg_midb_len == 0)
        return 0;
    memcpy(out, chg_midb, chg_midb_len);
    return chg_midb_len;
}

bool pdinfo_be_ident(pi_ident_t *out)
{
    if (!pdinfo_id_on() || chg_id.n == 0)
        return false;
    *out = chg_id;
    return true;
}

/* ================= 上位机 ================= */

/*
 * flags u8：bit0 透传开，bit1 身份透传开，bit2 设备身份，bit3 设备 SKEDB，bit4 电池容量，bit5 电池状态，
 *           bit6 充电器身份，bit7 充电器 SCEDB
 * flags2 u8：bit0 充电器 Source_Info，bit1 充电器 Status，bit2 充电器厂商信息
 * dev_vid u16, dev_pid u16, chg_vid u16, chg_pid u16（身份优先，其次扩展能力；0 = 未知）
 * bat_design u16, bat_full u16（0.1Wh）, bsdo u32
 * chg_pdp u8（充电器 SCEDB）, back_pdp u8, sido u32, chg_temp u8（Status 内部温度）
 * sink_modes u8, sink_op_pdp u8, sink_max_pdp u8（设备 SKEDB）
 * name_len u8, name[name_len]（充电器厂商信息字符串）
 */
uint8_t pdinfo_status(uint8_t *out)
{
    uint8_t *p = out;
    *p++ = (pdinfo_on() ? 0x01 : 0) | (pdinfo_id_on() ? 0x02 : 0) | (dev_id.n ? 0x04 : 0) | (dev_skedb_ok ? 0x08 : 0) |
           (dev_bcap_ok ? 0x10 : 0) | (dev_bsdo_ok ? 0x20 : 0) | (chg_id.n ? 0x40 : 0) | (chg_scedb_len ? 0x80 : 0);
    *p++ = (chg_sido_ok ? 0x01 : 0) | (chg_status_len ? 0x02 : 0) | (chg_midb_len ? 0x04 : 0);

    uint16_t dv = dev_id.n ? (uint16_t)dev_id.vdo[0] : dev_skedb_ok ? get16(&dev_skedb[EXT_VID]) : 0;
    uint16_t dp = dev_id.n >= 3 ? (uint16_t)(dev_id.vdo[2] >> 16) : dev_skedb_ok ? get16(&dev_skedb[EXT_PID]) : 0;
    uint16_t cv = chg_id.n ? (uint16_t)chg_id.vdo[0] : chg_scedb_len ? get16(&chg_scedb[EXT_VID]) : 0;
    uint16_t cp = chg_id.n >= 3 ? (uint16_t)(chg_id.vdo[2] >> 16) : chg_scedb_len ? get16(&chg_scedb[EXT_PID]) : 0;
    put16(p, dv), p += 2;
    put16(p, dp), p += 2;
    put16(p, cv), p += 2;
    put16(p, cp), p += 2;
    put16(p, dev_bcap_ok ? get16(&dev_bcap[4]) : 0), p += 2;
    put16(p, dev_bcap_ok ? get16(&dev_bcap[6]) : 0), p += 2;
    pd_put_u32(p, dev_bsdo_ok ? dev_bsdo : BSDO_INVALID_REF), p += 4;
    *p++ = chg_scedb_len ? chg_scedb[SCEDB_SPR_PDP] : 0;
    *p++ = back_pdp();
    pd_put_u32(p, chg_sido_ok ? chg_sido : 0), p += 4;
    *p++ = chg_status_len ? chg_status[0] : 0;
    *p++ = dev_skedb_ok ? dev_skedb[SKEDB_SINK_MODES] : 0;
    *p++ = dev_skedb_ok ? dev_skedb[SKEDB_SPR_PDP_MIN + 1] : 0;
    *p++ = dev_skedb_ok ? dev_skedb[SKEDB_SPR_PDP_MIN + 2] : 0;
    uint8_t name = chg_midb_len > MIDB_MIN_LEN ? chg_midb_len - MIDB_MIN_LEN : 0;
    while (name && chg_midb[MIDB_MIN_LEN + name - 1] == 0)
        name--;     /* 去掉结尾的 0 */
    *p++ = name;
    memcpy(p, &chg_midb[MIDB_MIN_LEN], name);
    p += name;
    return p - out;
}
