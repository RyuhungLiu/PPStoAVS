#include "bridge.h"
#include "analog.h"
#include "board.h"
#include "power_sw.h"
#include "timebase.h"
#include "pd_phy.h"

#define T_FRONT_REQ_START_MS    500     /* 前端忙（保活等）时等待其空闲的最长时间 */
#define T_FRONT_DONE_MS         1000    /* Accept + PS_RDY 的总时限（前端自身另有 tPSTransition 超时） */
#define T_VERIFY_MS             200

typedef struct
{
    pdo_type_t type;        /* FPDO 或 SPR_AVS_PDO */
    uint16_t   mv;          /* FPDO 电压 */
    uint16_t   ma;          /* FPDO 电流（已限流） */
    uint8_t    fe_pos;      /* 对应充电器 PDO 位置 */
} back_entry_t;

/* 后端能力表 */
static back_entry_t entries[PD_MAX_DATA_OBJS];
static uint32_t back_raw[PD_MAX_DATA_OBJS];
static uint8_t back_n;
static bool avs_enabled;
static uint16_t avs_max_mv, avs_ma15, avs_ma20;
static uint8_t pps_pos;
static pdo_t pps;

static bool back_caps_dirty;
static bool back_hard_reset;

/* 后端当前合约 */
static bool back_has_contract;
static uint16_t back_mv;
static pdo_type_t back_type;

/* 进行中的转换 */
static enum { TR_IDLE, TR_FRONT_REQ, TR_FRONT_WAIT, TR_VERIFY } tr_state = TR_IDLE;
static fe_target_t tr_target;
static pdo_type_t tr_back_type;
static uint16_t tr_mv;
static uint32_t tr_ts, tr_sample_ts;
static uint8_t tr_stable;

/* 监测 */
static bool want_front_5v;
static uint32_t mon_ts;
static uint8_t vbus_bad_count;
static bool ocp_active;
static uint32_t ocp_ts;

static uint16_t min_u16(uint16_t a, uint16_t b)
{
    return a < b ? a : b;
}

static bool vbus_in_window(uint16_t v, uint16_t target)
{
    uint32_t lo = (uint32_t)target * (100 - VBUS_TOLERANCE_PCT) / 100;
    uint32_t hi = (uint32_t)target * (100 + VBUS_TOLERANCE_PCT) / 100;
    return v >= lo && v <= hi;
}

static void add_entry(pdo_type_t type, uint16_t mv, uint16_t ma, uint8_t fe_pos, uint32_t raw)
{
    if (back_n >= PD_MAX_DATA_OBJS)
        return;
    entries[back_n].type = type;
    entries[back_n].mv = mv;
    entries[back_n].ma = ma;
    entries[back_n].fe_pos = fe_pos;
    back_raw[back_n] = raw;
    back_n++;
}

/* 选 PPS：先要求覆盖 20V，其次覆盖 15V；同档内取电流最大；最低电压须 ≤ 9V */
static void select_pps(const pdo_t *c, uint8_t n)
{
    uint8_t best_tier = 0;
    pps_pos = 0;
    for (uint8_t i = 0; i < n; i++)
    {
        if (c[i].type != PPS_PDO || c[i].min_mv > 9000)
            continue;
        uint8_t tier = (c[i].max_mv >= 20000) ? 2 : (c[i].max_mv >= 15000) ? 1 : 0;
        if (tier == 0)
            continue;
        if (tier > best_tier || (tier == best_tier && c[i].max_ma > pps.max_ma))
        {
            best_tier = tier;
            pps = c[i];
            pps_pos = i + 1;
        }
    }
}

/*
 * 规则（PD R3.2 表 3-1/3-2 注释、§4.1.3.2.4、表 4-3；2026-09-23 确认）：
 *  - 透传 Fixed（≤20V，≤3A）
 *  - 仅当有 15V Fixed 时才提供 SPR AVS
 *  - AVS 9~15V 电流 = 15V Fixed 电流，15~20V 电流 = 20V Fixed 电流，
 *    两者都取 min(充电器该档 Fixed 电流, 所选 PPS 电流, 3A)
 *  - 所选 PPS 最高不到 20V 时不提供 20V Fixed，AVS 只到 15V
 */
static void rebuild_back_caps(void)
{
    back_n = 0;
    avs_enabled = false;
    pps_pos = 0;

    uint8_t n;
    const pdo_t *c = fe_caps(&n);

    if (fe_is_legacy() || n == 0)
    {
        uint16_t ma = min_u16(fe_is_legacy() ? fe_legacy_current_ma() : 500, MAX_OUTPUT_CURRENT_MA);
        add_entry(FPDO, 5000, ma, 1, pd_build_fixed_pdo(5000, ma, 0));
#ifdef FE_DIAG
        /* 诊断（FE_DIAG）：把前端计数编码进 5.05~5.30V 的电流字段（10mA 单位） */
        const pd_phy_t *d = &pd_phy_fe;
        (void)d;
        const uint16_t *v = fe_dbg_v;
        for (uint8_t i = 0; i < 6; i++)
        {
            uint16_t x = v[i] > 1023 ? 1023 : v[i];
            add_entry(FPDO, 5050 + i * 50, 0, 1, pd_build_fixed_pdo(5050 + i * 50, 0, 0) | x);
        }
#endif
        return;
    }

    select_pps(c, n);

    bool has15 = false, has20 = false;
    for (uint8_t i = 0; i < n; i++)
    {
        if (c[i].type == FPDO && c[i].max_mv == 15000)
            has15 = true;
        if (c[i].type == FPDO && c[i].max_mv == 20000)
            has20 = true;
    }

    avs_enabled = pps_pos != 0 && has15;
    avs_max_mv = (avs_enabled && pps.max_mv >= 20000 && has20) ? 20000 : 15000;
    bool drop20 = avs_enabled && avs_max_mv < 20000;

    uint8_t max_fixed = avs_enabled ? PD_MAX_DATA_OBJS - 1 : PD_MAX_DATA_OBJS;
    avs_ma15 = avs_ma20 = 0;
    for (uint8_t i = 0; i < n && back_n < max_fixed; i++)
    {
        if (c[i].type != FPDO || c[i].max_mv > MAX_OUTPUT_VOLTAGE_MV)
            continue;
        uint16_t mv = c[i].max_mv;
        if (mv == 20000 && drop20)
            continue;

        uint16_t ma = min_u16(c[i].max_ma, MAX_OUTPUT_CURRENT_MA);
        if (avs_enabled && (mv == 15000 || mv == 20000))
        {
            ma = min_u16(ma, pps.max_ma);
            if (mv == 15000)
                avs_ma15 = ma;
            else
                avs_ma20 = ma;
        }

        /* PDO1 只保留 Unconstrained Power 标志，其余角色/能力位全部为 0 */
        uint32_t flags = (back_n == 0 && fe_caps_unconstrained()) ? (1u << 27) : 0;
        add_entry(FPDO, mv, ma, i + 1, pd_build_fixed_pdo(mv, ma, flags));
    }

    if (avs_enabled)
    {
        if (avs_max_mv < 20000)
            avs_ma20 = 0;
        add_entry(SPR_AVS_PDO, 0, 0, pps_pos, pd_build_spr_avs_apdo(avs_ma15, avs_ma20));
    }
}

static fe_target_t front_5v_target(void)
{
    uint8_t n;
    const pdo_t *c = fe_caps(&n);
    fe_target_t t = {1, FPDO, 5000, n ? c[0].max_ma : 500};
    return t;
}

static const fe_target_t *front_contract_if_idle(void)
{
    return fe_is_ready() ? fe_contract() : NULL;
}

fe_target_t bridge_on_front_caps(void)
{
    uint32_t old_raw[PD_MAX_DATA_OBJS];
    uint8_t old_n = back_n;
    memcpy(old_raw, back_raw, sizeof(old_raw));

    rebuild_back_caps();
    if (old_n != back_n || memcmp(old_raw, back_raw, back_n * 4) != 0)
        back_caps_dirty = true;

    fe_target_t t = front_5v_target();
    if (!back_has_contract || fe_is_legacy())
        return t;

    /* DPS：尽量保持设备当前电压，电流按新能力；保持不了就回 5V 并让后端 Hard Reset */
    uint8_t n;
    const pdo_t *c = fe_caps(&n);
    bool kept = false;
    if (back_type == FPDO)
    {
        for (uint8_t i = 0; i < back_n; i++)
        {
            if (entries[i].type == FPDO && entries[i].mv == back_mv)
            {
                t.pos = entries[i].fe_pos;
                t.type = FPDO;
                t.mv = back_mv;
                t.ma = c[t.pos - 1].max_ma;
                kept = true;
                break;
            }
        }
    }
    else if (avs_enabled && back_mv >= 9000 && back_mv <= avs_max_mv)
    {
        t.pos = pps_pos;
        t.type = PPS_PDO;
        t.mv = back_mv;
        t.ma = pps.max_ma;
        kept = true;
    }

    if (!kept)
    {
        back_hard_reset = true;
        back_has_contract = false;
        t = front_5v_target();
    }
    return t;
}

void bridge_on_front_reset(void)
{
    if (back_has_contract || tr_state != TR_IDLE)
        back_hard_reset = true;
    back_has_contract = false;
}

bool bridge_front_ready_for_vsafe5v(void)
{
    if (fe_is_legacy())
        return true;
    const fe_target_t *c = front_contract_if_idle();
    return c && c->type == FPDO && c->mv == 5000 && tr_state == TR_IDLE && !want_front_5v;
}

uint8_t bridge_back_caps(uint32_t *pdos)
{
    if (back_n == 0)
        rebuild_back_caps();
    memcpy(pdos, back_raw, back_n * 4);
    back_caps_dirty = false;
    return back_n;
}

bool bridge_eval_back_request(uint32_t rdo)
{
    uint8_t pos = pd_rdo_pos(rdo);
    if (pos == 0 || pos > back_n)
        return false;

    uint8_t n;
    const pdo_t *c = fe_caps(&n);
    const back_entry_t *e = &entries[pos - 1];

    if (e->type == FPDO)
    {
        if (pd_rdo_fixed_op_ma(rdo) > e->ma)
            return false;
        tr_back_type = FPDO;
        tr_mv = e->mv;
        tr_target.pos = e->fe_pos;
        tr_target.type = FPDO;
        tr_target.mv = e->mv;
        tr_target.ma = (n && !fe_is_legacy()) ? c[e->fe_pos - 1].max_ma : e->ma;
        return true;
    }

    /* SPR AVS：电压 25mV 单位、100mV 步进；按 9~avs_max 校验 */
    uint32_t mv = pd_rdo_avs_mv(rdo) / 100 * 100;
    uint16_t op_ma = pd_rdo_avs_ma(rdo);
    if (mv < 9000 || mv > avs_max_mv)
        return false;
    uint16_t limit = (mv < 15000) ? avs_ma15 : (mv > 15000) ? avs_ma20
                                             : (avs_ma15 > avs_ma20 ? avs_ma15 : avs_ma20);
    if (op_ma > limit)
        return false;

    tr_back_type = SPR_AVS_PDO;
    tr_mv = mv;
    tr_target.pos = pps_pos;
    tr_target.type = PPS_PDO;
    tr_target.mv = mv;
    tr_target.ma = pps.max_ma;
    return true;
}

void bridge_start_transition(void)
{
    const fe_target_t *c = fe_contract();
    tr_ts = millis();
    if (fe_is_legacy() ||
        (c && c->type == tr_target.type && c->pos == tr_target.pos && c->mv == tr_target.mv))
    {
        tr_state = TR_VERIFY;
        tr_stable = 0;
        tr_sample_ts = 0;
    }
    else
    {
        tr_state = TR_FRONT_REQ;
    }
}

bridge_result_t bridge_poll_transition(void)
{
    uint32_t now = millis();
    switch (tr_state)
    {
    case TR_FRONT_REQ:
        if (fe_request(&tr_target))
        {
            tr_state = TR_FRONT_WAIT;
            tr_ts = now;
        }
        else if (now - tr_ts > T_FRONT_REQ_START_MS)
        {
            tr_state = TR_IDLE;
            return BRIDGE_FAIL;
        }
        return BRIDGE_PENDING;

    case TR_FRONT_WAIT:
    {
        fe_req_status_t s = fe_request_status();
        if (s == FE_REQ_OK)
        {
            tr_state = TR_VERIFY;
            tr_ts = now;
            tr_stable = 0;
            tr_sample_ts = 0;
        }
        else if (s == FE_REQ_FAIL || now - tr_ts > T_FRONT_DONE_MS)
        {
            tr_state = TR_IDLE;
            return BRIDGE_FAIL;
        }
        return BRIDGE_PENDING;
    }

    case TR_VERIFY:
        if (now - tr_sample_ts >= VBUS_SAMPLE_INTERVAL_MS)
        {
            tr_sample_ts = now;
            tr_stable = vbus_in_window(analog_vbus_mv(), tr_mv) ? tr_stable + 1 : 0;
            if (tr_stable >= VBUS_STABLE_SAMPLES)
            {
                tr_state = TR_IDLE;
                return BRIDGE_OK;
            }
        }
        if (now - tr_ts > T_VERIFY_MS)
        {
            tr_state = TR_IDLE;
            return BRIDGE_FAIL;
        }
        return BRIDGE_PENDING;

    default:
        return BRIDGE_FAIL;
    }
}

void bridge_on_back_contract(void)
{
    back_has_contract = true;
    back_type = tr_back_type;
    back_mv = tr_mv;
    vbus_bad_count = 0;
    ocp_active = false;
}

void bridge_on_back_reset(void)
{
    back_has_contract = false;
    back_hard_reset = false;
    tr_state = TR_IDLE;
    want_front_5v = true;
}

bool bridge_take_back_caps_dirty(void)
{
    bool d = back_caps_dirty;
    back_caps_dirty = false;
    return d;
}

bool bridge_take_back_hard_reset(void)
{
    bool r = back_hard_reset;
    back_hard_reset = false;
    return r;
}

static void protection_trip(void)
{
    power_sw_set(false);
    back_has_contract = false;
    back_hard_reset = true;
}

void bridge_process(void)
{
    if (want_front_5v)
    {
        const fe_target_t *c = fe_contract();
        if (fe_is_legacy() || (c && c->type == FPDO && c->mv == 5000) || !c)
        {
            want_front_5v = false;
        }
        else if (fe_is_ready())
        {
            fe_target_t t = front_5v_target();
            if (fe_request(&t))
                want_front_5v = false;
        }
    }

    uint32_t now = millis();
    if (!back_has_contract || !power_sw_is_on() || tr_state != TR_IDLE ||
        !(fe_is_ready() || fe_is_legacy()))
    {
        vbus_bad_count = 0;
        ocp_active = false;
        return;
    }
    if (now - mon_ts < VBUS_SAMPLE_INTERVAL_MS)
        return;
    mon_ts = now;

    /* 过/欠压：±5%，连续 3 次 */
    vbus_bad_count = vbus_in_window(analog_vbus_mv(), back_mv) ? 0 : vbus_bad_count + 1;
    if (vbus_bad_count >= VBUS_STABLE_SAMPLES)
    {
        protection_trip();
        return;
    }

    /* 过流：> 3.5A 持续 50ms */
    if (analog_current_ma() > OCP_CURRENT_MA)
    {
        if (!ocp_active)
        {
            ocp_active = true;
            ocp_ts = now;
        }
        else if (now - ocp_ts >= OCP_TIME_MS)
        {
            protection_trip();
        }
    }
    else
    {
        ocp_active = false;
    }
}
