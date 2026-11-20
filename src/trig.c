/* SPDX-License-Identifier: MPL-2.0 */
#include "trig.h"
#include "afc.h"
#include "analog.h"
#include "board.h"
#include "cfg.h"
#include "evlog.h"
#include "fe_ufcs.h"
#include "power_sw.h"
#include "qc.h"
#include "timebase.h"

#define T_UFCS_CAPS_MS      2500    /* UFCS 会话建立后等输出模式并完成请求 */
#define T_PD_CAPS_MS        6000    /* PD：这么久没有能力报文 = 不是 PD 充电器 */
#define T_REQ_MS            3000    /* 能力报文选好档后，这么久没到目标 = 请求失败 */
#define T_SETTLE_MS         500     /* 合约 / 电平到位后，电压进入窗口的最长等待 */
#define T_SAMPLE_MS         2
#define N_GOOD              3       /* 连续 3 次在窗口内 = 到位 */
#define N_BAD               3       /* 运行中连续 3 次超出窗口 = 故障 */
#define N_LOST              3       /* 协议中断最多重新协商的次数 */
#define T_5V_RETRY_MS       500

enum
{
    PICK_NONE,
    PICK_YES,
    PICK_NO,
};

static bool act;
static trig_state_t st;
static uint8_t proto;               /* 当前（或最后）尝试的协议 TRIGP_* */
static uint8_t reason;
static fe_target_t want;            /* PD / UFCS：选中的档位 */
static uint8_t pick;
static uint32_t ts, sample_ts;
static uint8_t good, bad, lost;
static bool pd_wanted;
static bool need_5v;                /* 故障 / 放弃后要把充电器带回 5V */
static uint32_t five_ts;
static uint8_t afc_five;            /* AFC 表里的 5V 字节（没有就用 0x00 = 5V 0.75A） */

bool trig_active(void)
{
    return act;
}

bool trig_pd_wanted(void)
{
    return pd_wanted;
}

static void log_trig(uint8_t s, uint8_t r)
{
    ev_trig_t e = {s, proto, r, cfg_trig()->mv, analog_vbus_mv(), pick == PICK_YES ? want.ma : 0};
    evlog_add(EV_TRIG, &e, sizeof(e));
}

static void set_state(trig_state_t s, uint8_t r)
{
    st = s;
    reason = r;
    ts = millis();
    good = bad = 0;
    log_trig(s, r);
}

/* 记一笔「跳过 / 失败」，状态不变（接着试下一个协议） */
static void skip(uint8_t p, uint8_t r)
{
    proto = p;
    log_trig(TRIG_SKIPPED, r);
}

static bool in_window(uint16_t v)
{
    const cfg_t *c = cfg();
    uint32_t t = cfg_trig()->mv;
    return v <= t + t * c->ovp_pct / 100 && v >= t - t * c->uvp_pct / 100;
}

/* 阻塞（启动阶段）：等电压连续 N_GOOD 次进入窗口 */
static bool settled(uint32_t timeout_ms)
{
    uint32_t t0 = millis();
    uint8_t k = 0;
    while (millis() - t0 < timeout_ms)
    {
        k = in_window(analog_vbus_mv()) ? k + 1 : 0;
        if (k >= N_GOOD)
            return true;
        delay_ms(T_SAMPLE_MS);
    }
    return false;
}

/* 该协议开启、目标合法才试；否则记录原因并跳过 */
static bool may_try(uint8_t p)
{
    const cfg_trig_t *t = cfg_trig();
    if (!(t->protos & p))
    {
        skip(p, TRR_OFF);
        return false;
    }
    if (!cfg_trig_mv_ok(p, t->mv))
    {
        skip(p, TRR_MV);
        return false;
    }
    proto = p;
    pick = PICK_NONE;
    set_state(TRIG_NEGOTIATING, TRR_NONE);
    return true;
}

void trig_boot(void)
{
    act = (cfg_trig()->flags & TRIGF_ON) != 0;
    if (!act)
        return;
    power_sw_lock();
    evlog_force();
}

/* ---------------- 选档（UFCS 合成能力 / PD 能力报文） ---------------- */

static bool choose(const pdo_t *c, uint8_t n, fe_target_t *t)
{
    const cfg_trig_t *g = cfg_trig();
    uint16_t mv = g->mv;
    bool ufcs = feu_active();
    bool adj = ufcs || (g->flags & TRIGF_PD_ADJ);       /* UFCS 的模式本来就是可调的 */
    int8_t k = -1;
    if (n > PD_MAX_DATA_OBJS)
        n = PD_MAX_DATA_OBJS;                            /* 不用 EPR 档 */
    for (uint8_t i = 0; i < n && k < 0; i++)
        if (c[i].type == PDO_FIXED && c[i].max_mv == mv)
            k = (int8_t)i;
    if (k < 0 && adj && mv % 20 == 0)
        for (uint8_t i = 0; i < n; i++)
            if (c[i].type == PDO_PPS && c[i].min_mv <= mv && mv <= c[i].max_mv && (k < 0 || c[i].max_ma > c[k].max_ma))
                k = (int8_t)i;
    if (k < 0 && adj && !ufcs && mv % 100 == 0)
        for (uint8_t i = 0; i < n && k < 0; i++)
            if (c[i].type == PDO_SPR_AVS && mv >= 9000 && mv <= c[i].max_mv)
                k = (int8_t)i;
    if (k < 0)
        return false;
    uint16_t ma = c[k].type == PDO_SPR_AVS && mv > 15000 ? c[k].max_ma_20v : c[k].max_ma;
    if (g->ma && g->ma < ma)
        ma = g->ma;
    *t = (fe_target_t){(uint8_t)(k + 1), c[k].type, mv, ma};
    return true;
}

static fe_target_t five_target(void)
{
    uint8_t n;
    const pdo_t *c = fe_caps(&n);
    uint16_t ma = n ? c[0].max_ma : 900;
    return (fe_target_t){1, PDO_FIXED, 5000, ma > 3000 ? 3000 : ma};
}

fe_target_t trig_on_caps(void)
{
    uint8_t p = feu_active() ? TRIGP_UFCS : TRIGP_PD;
    if (p != proto || (st != TRIG_NEGOTIATING && st != TRIG_LOST && st != TRIG_ACTIVE))
        return five_target();       /* 故障 / 放弃后：只留 5V */
    uint8_t n;
    const pdo_t *c = fe_caps(&n);
    if (choose(c, n, &want))
    {
        pick = PICK_YES;
        if (st == TRIG_LOST)
            set_state(TRIG_NEGOTIATING, TRR_NONE);
        ts = millis();
        good = 0;
        return want;
    }
    pick = PICK_NO;
    return five_target();
}

void trig_on_front_reset(void)
{
    if (!act || (st != TRIG_ACTIVE && st != TRIG_NEGOTIATING))
        return;
    pick = PICK_NONE;
    if (++lost > N_LOST)
        set_state(TRIG_FAULT, TRR_RETRIES);
    else
        set_state(TRIG_LOST, TRR_LOST);
}

/* ---------------- 启动阶段：UFCS / QC / AFC（阻塞，看门狗启动前） ---------------- */

/* UFCS 失败后把充电器带回 5V 再关闭会话 */
static void ufcs_give_up(void)
{
    if (feu_is_ready())
    {
        const fe_target_t *c = feu_contract();
        if (c && c->mv != 5000)
        {
            fe_target_t f = five_target();
            feu_request(&f);
            uint32_t t0 = millis();
            while (millis() - t0 < 1000 && feu_request_status() == FE_REQ_BUSY && feu_process())
                ;
        }
    }
    feu_stop();
}

static bool run_ufcs(void)
{
    if (!feu_start())
    {
        skip(TRIGP_UFCS, TRR_NO_RESP);
        return false;
    }
    uint32_t t0 = millis();
    while (millis() - t0 < T_UFCS_CAPS_MS)
    {
        if (!feu_process())
        {
            skip(TRIGP_UFCS, TRR_LOST);
            feu_stop();
            return false;
        }
        if (pick == PICK_NO)
        {
            skip(TRIGP_UFCS, TRR_NO_LEVEL);
            ufcs_give_up();
            return false;
        }
        const fe_target_t *c = feu_contract();
        if (pick == PICK_YES && feu_is_ready() && c && c->pos == want.pos && c->mv == want.mv)
        {
            if (settled(T_SETTLE_MS))
            {
                set_state(TRIG_ACTIVE, TRR_NONE);
                return true;
            }
            skip(TRIGP_UFCS, TRR_NOT_REACHED);
            ufcs_give_up();
            return false;
        }
    }
    skip(TRIGP_UFCS, pick == PICK_YES ? TRR_REJECT : TRR_NO_RESP);
    ufcs_give_up();
    return false;
}

static void qc_wait(uint32_t ms)
{
    uint32_t t0 = millis();
    while (qc_busy() && millis() - t0 < ms)
        qc_process();
}

static bool run_qc(void)
{
    uint16_t mv = cfg_trig()->mv;
    bool fixed = mv == 5000 || mv == 9000 || mv == 12000;
    qc_set(mv, !fixed);         /* QC2 直接设定压档；其他电压在 5V 进连续模式后逐步升到目标，不会越过目标 */
    qc_wait(2000);
    if (settled(T_SETTLE_MS))
    {
        set_state(TRIG_ACTIVE, TRR_NONE);
        return true;
    }
    skip(TRIGP_QC, TRR_NOT_REACHED);
    qc_set(5000, false);
    qc_wait(500);
    return false;
}

static bool run_afc(void)
{
    uint16_t mv = cfg_trig()->mv;
    uint8_t raw[8];
    uint8_t n = afc_query(raw, sizeof(raw));    /* 送一个充电器不支持的 5V 请求取表，电压不变 */
    if (!n)
    {
        skip(TRIGP_AFC, TRR_NO_RESP);
        return false;
    }
    int8_t k = -1;
    afc_five = 0x00;
    for (uint8_t i = 0; i < n; i++)
    {
        if (afc_mv(raw[i]) == mv && (k < 0 || afc_ma(raw[i]) > afc_ma(raw[k])))
            k = (int8_t)i;
        if (afc_mv(raw[i]) == 5000)
            afc_five = raw[i];
    }
    if (k < 0)
    {
        skip(TRIGP_AFC, TRR_NO_LEVEL);
        return false;
    }
    afc_set(raw[k]);
    uint32_t t0 = millis();
    while (afc_busy() && millis() - t0 < 1500)
        afc_process();
    if (afc_ok() && settled(T_SETTLE_MS))
    {
        set_state(TRIG_ACTIVE, TRR_NONE);
        return true;
    }
    skip(TRIGP_AFC, TRR_NOT_REACHED);
    afc_set(afc_five);
    t0 = millis();
    while (afc_busy() && millis() - t0 < 1500)
        afc_process();
    return false;
}

/* D± 被前一个协议动过：拉到 0V 让充电器退出 HVDCP / UFCS，再做 BC1.2→HVDCP 握手 */
static uint16_t handshake(bool reset)
{
    if (reset)
    {
        qc_lines_low();
        delay_ms(100);
    }
    return qc_handshake();
}

uint8_t trig_run_dcp(void)
{
    if (!act)
        return 0;
    bool touched = false;
    if (may_try(TRIGP_UFCS))
    {
        if (run_ufcs())
            return TRIGP_UFCS;
        touched = true;
    }
    uint16_t hs = 0;
    bool shook = false;
    if (may_try(TRIGP_QC))
    {
        hs = handshake(touched);
        shook = true;
        if (!hs)
            skip(TRIGP_QC, TRR_NO_RESP);
        else if (run_qc())
            return TRIGP_QC;
        touched = true;
    }
    if (may_try(TRIGP_AFC))
    {
        /* QC 试过 D+ 3.3V 等电平，AFC 充电器可能已离开 HVDCP：重新握手（与正式 QC→AFC 流程相同） */
        if (!shook || hs)
            hs = handshake(touched);
        if (!hs)
            skip(TRIGP_AFC, TRR_NO_RESP);
        else if (run_afc())
            return TRIGP_AFC;
    }
    if (shook || hs)
        qc_release();
    pd_wanted = may_try(TRIGP_PD);
    if (!pd_wanted)
    {
        proto = 0;
        set_state(TRIG_UNAVAILABLE, TRR_NONE);
    }
    return 0;
}

/* ---------------- 主循环 ---------------- */

static void drive_5v(uint32_t now)
{
    if (!need_5v || now - five_ts < T_5V_RETRY_MS)
        return;
    five_ts = now;
    if (proto == TRIGP_QC)
    {
        if (!qc_busy())
        {
            if (analog_vbus_mv() > 5500)
                qc_set(5000, false);
            else
                need_5v = false;
        }
        return;
    }
    if (proto == TRIGP_AFC)
    {
        if (!afc_busy())
        {
            if (analog_vbus_mv() > 5500)
                afc_set(afc_five);
            else
                need_5v = false;
        }
        return;
    }
    const fe_target_t *c = fe_contract();
    if (!c || c->mv == 5000)
    {
        need_5v = false;
        return;
    }
    if (fe_is_ready())
    {
        fe_target_t f = five_target();
        fe_request(&f);
    }
}

static void give_up(trig_state_t s, uint8_t r)
{
    set_state(s, r);
    pick = PICK_NONE;
    need_5v = true;
    five_ts = millis() - T_5V_RETRY_MS;
}

void trig_process(void)
{
    if (!act)
        return;
    uint32_t now = millis();
    if (proto == TRIGP_QC)
        qc_process();
    else if (proto == TRIGP_AFC)
        afc_process();

    if ((proto == TRIGP_PD || proto == TRIGP_UFCS) && (st == TRIG_NEGOTIATING || st == TRIG_LOST))
    {
        if (proto == TRIGP_PD && fe_is_legacy())
            give_up(TRIG_UNAVAILABLE, TRR_NO_RESP);
        else if (proto == TRIGP_PD && pick == PICK_NONE && !fe_caps_available() && now - ts > T_PD_CAPS_MS)
            give_up(TRIG_UNAVAILABLE, TRR_NO_RESP);
        else if (pick == PICK_NONE && fe_caps_available() && fe_is_ready())
        {
            /* 能力报文早于本状态到达（前端虚拟 E-Marker 提前启动 PD、或中断后已回 5V）：直接按现有能力请求 */
            uint8_t n;
            const pdo_t *c = fe_caps(&n);
            if (!choose(c, n, &want))
                pick = PICK_NO;
            else if (fe_request(&want))
            {
                pick = PICK_YES;
                ts = now;
                good = 0;
            }
        }
        else if (pick == PICK_NO)
            give_up(TRIG_UNAVAILABLE, TRR_NO_LEVEL);
        else if (pick == PICK_YES)
        {
            const fe_target_t *c = fe_contract();
            if (fe_is_ready() && c && c->pos == want.pos && c->mv == want.mv)
            {
                if (now - sample_ts >= T_SAMPLE_MS)
                {
                    sample_ts = now;
                    good = in_window(analog_vbus_mv()) ? good + 1 : 0;
                    if (good >= N_GOOD)
                        set_state(TRIG_ACTIVE, TRR_NONE);
                }
                if (st != TRIG_ACTIVE && now - ts > T_REQ_MS + T_SETTLE_MS)
                    give_up(TRIG_UNAVAILABLE, TRR_NOT_REACHED);
            }
            else if (now - ts > T_REQ_MS)
                give_up(TRIG_UNAVAILABLE, TRR_REJECT);
        }
    }
    else if (st == TRIG_ACTIVE && now - sample_ts >= T_SAMPLE_MS)
    {
        sample_ts = now;
        bad = in_window(analog_vbus_mv()) ? 0 : bad + 1;
        if (bad >= N_BAD)
            give_up(TRIG_FAULT, TRR_WINDOW);
    }
    drive_5v(now);
}

uint8_t trig_status(uint8_t *buf)
{
    const cfg_trig_t *t = cfg_trig();
    uint16_t v = analog_vbus_mv();
    uint16_t ma = pick == PICK_YES ? want.ma : 0;
    uint8_t n = 0;
    buf[n++] = act;
    buf[n++] = (uint8_t)st;
    buf[n++] = proto;
    buf[n++] = reason;
    buf[n++] = t->mv & 0xFF;
    buf[n++] = t->mv >> 8;
    buf[n++] = v & 0xFF;
    buf[n++] = v >> 8;
    buf[n++] = ma & 0xFF;
    buf[n++] = ma >> 8;
    buf[n++] = lost;
    buf[n++] = power_sw_locked();
    return n;
}
