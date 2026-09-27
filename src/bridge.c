#include "bridge.h"
#include "analog.h"
#include "board.h"
#include "cfg.h"
#include "evlog.h"
#include "pd_phy.h"
#include "power_sw.h"
#include "timebase.h"

#define T_FRONT_REQ_START_MS    500     /* 前端忙（保活等）时等待其空闲的最长时间 */
#define T_FRONT_DONE_MS         1000    /* Accept + PS_RDY 的总时限（前端自身另有 tPSTransition 超时） */
#define T_VERIFY_MS             200
#define T_RETRY_BUDGET_MS       300     /* 前端被打断后重发的截止时间（设备 tPSTransition ≥ 450ms） */

typedef struct
{
    pdo_type_t type;        /* 后端 PDO 类型：FPDO / PPS_PDO / SPR_AVS_PDO */
    pdo_type_t fe_type;     /* 前端向充电器请求的类型（12V 转换、AVS 平移时与 type 不同） */
    uint8_t    fe_pos;      /* 对应充电器 PDO 位置（1-based） */
    uint16_t   min_mv;      /* PPS 最低电压；AVS 为 9000 */
    uint16_t   mv;          /* Fixed 电压；PPS/AVS 最高电压 */
    uint16_t   ma;          /* 已限流；AVS 为 9~15V 电流 */
    uint16_t   ma20;        /* 仅 AVS：15~20V 电流 */
} back_entry_t;

/* 后端能力表 */
static back_entry_t entries[PD_MAX_DATA_OBJS];
static uint32_t back_raw[PD_MAX_DATA_OBJS];
static uint8_t back_n;
static uint8_t logged[PD_MAX_DATA_OBJS * 5];    /* 最近记录的能力（PDO + 来源），内容变化才再记录 */
static uint8_t logged_len = 0xFF;

static bool back_caps_dirty;
static bool back_hard_reset;

/* Lab：后端 AVS 二次握手（CFG_BE_AVS_2ND）——先给 PPS，设备在 Sink_Capabilities_Extended 中声明支持 AVS 后才给 AVS */
static bool avs_held;           /* 本可提供 AVS，但尚未确认设备支持 */
static bool rear_queried;       /* 本次连接已查询过设备 */
static bool rear_avs_ok;        /* 设备已声明支持 AVS */

/* 后端当前合约 */
static bool back_has_contract;
static uint16_t back_mv, back_ma;
static pdo_type_t back_type;
static uint32_t back_rdo;
static uint32_t back_rdo_logged;    /* 最近一次记录的成功请求，重复请求（PPS 保活）不再记录 */

/* 进行中的转换 */
static enum { TR_IDLE, TR_FRONT_REQ, TR_FRONT_WAIT, TR_VERIFY } tr_state = TR_IDLE;
static fe_target_t tr_target;
static pdo_type_t tr_back_type;
static uint16_t tr_mv, tr_op_ma;
static uint32_t tr_rdo;
static uint32_t tr_start_ts;        /* 回复设备 Accept 的时刻 */
static bool tr_front_sent;          /* 本次转换向充电器发过请求 */
static uint8_t tr_txwait;

static bool map_back_request(uint32_t rdo);
static uint32_t tr_ts, tr_sample_ts;
static uint8_t tr_stable;

/* 监测 */
static bool want_front_5v;
static bool front_5v_pending;       /* 已发出回 5V 请求，等待结果以记录 */
static uint32_t mon_ts;
static uint8_t vbus_bad_count;
static uint8_t vbus_bad_kind;
static bool ocp_active;
static uint32_t ocp_ts;

static uint16_t min_u16(uint16_t a, uint16_t b)
{
    return a < b ? a : b;
}

static bool vbus_over(uint16_t v, uint16_t target)
{
    return v > (uint32_t)target * (100 + cfg()->ovp_pct) / 100;
}

static bool vbus_under(uint16_t v, uint16_t target)
{
    return v < (uint32_t)target * (100 - cfg()->uvp_pct) / 100;
}

static void add_entry(const back_entry_t *e, uint32_t raw)
{
    if (back_n >= PD_MAX_DATA_OBJS)
        return;
    entries[back_n] = *e;
    back_raw[back_n] = raw;
    back_n++;
}

static bool fixed_hidden(uint16_t mv)
{
    uint8_t h = cfg()->hide_fixed;
    return (mv == 9000 && (h & CFG_HIDE_9V)) || (mv == 12000 && (h & CFG_HIDE_12V)) ||
           (mv == 15000 && (h & CFG_HIDE_15V)) || (mv == 20000 && (h & CFG_HIDE_20V));
}

static int8_t find_mv(const back_entry_t *list, uint8_t n, uint16_t mv)
{
    for (uint8_t k = 0; k < n; k++)
    {
        if (list[k].mv == mv)
            return k;
    }
    return -1;
}

/* 按 mv 升序插入；同电压已存在则不插入 */
static void insert_sorted(back_entry_t *list, uint8_t *n, const back_entry_t *e)
{
    if (*n >= PD_MAX_DATA_OBJS || find_mv(list, *n, e->mv) >= 0)
        return;
    uint8_t i = *n;
    while (i > 0 && list[i - 1].mv > e->mv)
    {
        list[i] = list[i - 1];
        i--;
    }
    list[i] = *e;
    (*n)++;
}

/* AVS 可达档位：2 = 到 20V（最高电压设为 20V 时），1 = 到 15V，0 = 不可用 */
static uint8_t avs_tier(uint16_t max_mv)
{
    return (max_mv >= 20000 && cfg()->max_mv >= 20000) ? 2 : (max_mv >= 15000) ? 1 : 0;
}

static int8_t find_native_avs(const pdo_t *c, uint8_t n)
{
    for (uint8_t i = 0; i < n; i++)
    {
        if (c[i].type == SPR_AVS_PDO && c[i].max_ma > 0)
            return i;
    }
    return -1;
}

/*
 * 后端能力（PD R3.2 v1.2 §6.4.1.1、表 3-1/3-2 注释、§4.1.3.2.4、表 4-3）：
 *  - Fixed：透传 ≤ 最高电压、未隐藏的档位，电流 ≤ 电流上限
 *  - AVS（模式 a/c）：仅当提供 15V Fixed 时。来源优先充电器原生 AVS（原样透传）；没有、或可达档位低于 PPS 时
 *    由 PPS 平移（最低电压 ≤ 9V，先要求到 20V，其次到 15V，同档取电流最大）。9~15V 电流 = 15V Fixed 电流，15~20V 电流 = 20V Fixed 电流，
 *    都取 min(充电器该档 Fixed 电流, 来源电流, 电流上限)；AVS 不到 20V 时不提供 20V Fixed
 *  - PPS（模式 b/c）：充电器 APDO 全部透传，最高电压与电流按设置截断，同一最高电压只保留一个
 *  - 12V 转换（CFG_FIX12）：充电器没有 12V 时由覆盖 12V 的 PPS（优先）或原生 AVS 提供
 *  - 个数 ≤ 7，优先级 Fixed > AVS > PPS（保留最高电压高的）> 12V 转换
 *  - 顺序：Fixed（电压升序）、AVS、PPS（最高电压升序）
 *  - Lab 后端二次握手：确认设备支持 AVS 之前不给 AVS、改给全部 PPS
 *  - Lab AVS 转 PPS（CFG_AVS_TO_PPS，模式 b/c）：充电器原生 AVS 另外提供为 5V~AVS 最高电压的 PPS，
 *    电流取两段 AVS 电流较小者；同一最高电压已有充电器 PPS 时以充电器 PPS 为准
 */
static void build_back_caps(void)
{
    const cfg_t *c = cfg();
    back_n = 0;
    avs_held = false;

    uint8_t n;
    const pdo_t *fc = fe_caps(&n);

    if (fe_is_legacy() || n == 0)
    {
        uint16_t ma = min_u16(fe_is_legacy() ? fe_legacy_current_ma() : 500, c->max_ma);
        back_entry_t e = {FPDO, FPDO, 1, 5000, 5000, ma, 0};
        add_entry(&e, pd_build_fixed_pdo(5000, ma, 0));
#ifdef FE_DIAG
        /* 诊断（FE_DIAG）：把前端计数编码进 5.05~5.30V 的电流字段（10mA 单位） */
        for (uint8_t i = 0; i < 6; i++)
        {
            uint16_t x = fe_dbg_v[i] > 1023 ? 1023 : fe_dbg_v[i];
            back_entry_t d = {FPDO, FPDO, 1, 5050 + i * 50, 5050 + i * 50, 0, 0};
            add_entry(&d, pd_build_fixed_pdo(5050 + i * 50, 0, 0) | x);
        }
#endif
        return;
    }

    bool want_avs = c->mode == CFG_MODE_AVS || c->mode == CFG_MODE_AVS_PPS;
    bool want_pps = c->mode == CFG_MODE_PPS || c->mode == CFG_MODE_AVS_PPS;

    /* 1. 充电器 Fixed */
    back_entry_t fixed[PD_MAX_DATA_OBJS];
    uint8_t nf = 0;
    for (uint8_t i = 0; i < n; i++)
    {
        uint16_t mv = fc[i].max_mv;
        if (fc[i].type != FPDO || mv > c->max_mv || fixed_hidden(mv))
            continue;
        back_entry_t e = {FPDO, FPDO, i + 1, mv, mv, min_u16(fc[i].max_ma, c->max_ma), 0};
        insert_sorted(fixed, &nf, &e);
    }

    /* 2. AVS */
    back_entry_t avs;
    bool has_avs = false;
    int8_t f15 = find_mv(fixed, nf, 15000);
    if (want_avs && f15 >= 0)
    {
        uint8_t pps_tier = 0, pps_i = 0;
        for (uint8_t i = 0; i < n; i++)
        {
            if (fc[i].type != PPS_PDO || fc[i].min_mv > 9000)
                continue;
            uint8_t tier = avs_tier(fc[i].max_mv);
            if (tier > pps_tier || (tier == pps_tier && tier > 0 && fc[i].max_ma > fc[pps_i].max_ma))
            {
                pps_tier = tier;
                pps_i = i;
            }
        }
        int8_t nat = find_native_avs(fc, n);
        uint8_t nat_tier = nat >= 0 ? avs_tier(fc[nat].max_mv) : 0;

        const pdo_t *src = NULL;
        uint8_t src_pos = 0;
        uint16_t s15 = 0, s20 = 0;
        if (nat_tier > 0 && nat_tier >= pps_tier)
        {
            src = &fc[nat];
            src_pos = nat + 1;
            s15 = src->max_ma;
            s20 = src->max_ma_20v;
        }
        else if (pps_tier > 0)
        {
            src = &fc[pps_i];
            src_pos = pps_i + 1;
            s15 = s20 = src->max_ma;
        }

        if (src)
        {
            int8_t f20 = find_mv(fixed, nf, 20000);
            uint16_t avs_max = (avs_tier(src->max_mv) == 2 && f20 >= 0) ? 20000 : 15000;
            bool drop20 = avs_max < 20000 && f20 >= 0;
            if (nf - drop20 < PD_MAX_DATA_OBJS && (c->flags & CFG_BE_AVS_2ND) && !rear_avs_ok)
            {
                avs_held = true;
            }
            else if (nf - drop20 < PD_MAX_DATA_OBJS)
            {
                if (drop20)
                {
                    for (uint8_t k = f20; k + 1 < nf; k++)
                        fixed[k] = fixed[k + 1];
                    nf--;
                }
                uint16_t ma15 = min_u16(fixed[f15].ma, s15);
                fixed[f15].ma = ma15;
                uint16_t ma20 = 0;
                if (avs_max == 20000)
                {
                    ma20 = min_u16(fixed[f20].ma, s20);
                    fixed[f20].ma = ma20;
                }
                back_entry_t e = {SPR_AVS_PDO, src->type, src_pos, 9000, avs_max, ma15, ma20};
                avs = e;
                has_avs = true;
            }
        }
    }

    /* 3. PPS 透传（二次握手第一阶段用 PPS 代替 AVS） */
    back_entry_t pps[PD_MAX_DATA_OBJS];
    uint8_t np = 0;
    if (want_pps || avs_held)
    {
        for (uint8_t i = 0; i < n; i++)
        {
            if (fc[i].type != PPS_PDO || fc[i].min_mv >= c->max_mv)
                continue;
            back_entry_t e = {PPS_PDO, PPS_PDO, i + 1, fc[i].min_mv, min_u16(fc[i].max_mv, c->max_mv),
                              min_u16(fc[i].max_ma, c->max_ma), 0};
            int8_t k = find_mv(pps, np, e.mv);
            if (k < 0)
                insert_sorted(pps, &np, &e);
            else if (e.ma > pps[k].ma || (e.ma == pps[k].ma && e.min_mv < pps[k].min_mv))
                pps[k] = e;
        }
        int8_t nat = find_native_avs(fc, n);
        if ((c->flags & CFG_AVS_TO_PPS) && nat >= 0 && fc[nat].max_mv > 9000)
        {
            const pdo_t *a = &fc[nat];
            uint16_t mv = min_u16(a->max_mv, c->max_mv);
            uint16_t ma = (mv > 15000 && a->max_ma_20v) ? min_u16(a->max_ma, a->max_ma_20v) : a->max_ma;
            back_entry_t e = {PPS_PDO, SPR_AVS_PDO, nat + 1, 5000, mv, min_u16(ma, c->max_ma), 0};
            insert_sorted(pps, &np, &e);    /* 同一最高电压已有充电器 PPS 则不插入 */
        }
        uint8_t room = PD_MAX_DATA_OBJS - nf - has_avs;
        if (np > room)
        {
            /* 位置不够：保留最高电压高的 */
            for (uint8_t k = 0; k < room; k++)
                pps[k] = pps[np - room + k];
            np = room;
        }
    }

    /* 4. 12V 转换 */
    if ((c->flags & CFG_FIX12) && c->max_mv >= 12000 && !fixed_hidden(12000) && find_mv(fixed, nf, 12000) < 0 &&
        nf + has_avs + np < PD_MAX_DATA_OBJS)
    {
        int8_t best = -1;
        for (uint8_t i = 0; i < n; i++)
        {
            if (fc[i].type == PPS_PDO && fc[i].min_mv <= 12000 && fc[i].max_mv >= 12000 &&
                (best < 0 || fc[i].max_ma > fc[best].max_ma))
                best = i;
        }
        if (best < 0)
            best = find_native_avs(fc, n);
        if (best >= 0)
        {
            back_entry_t e = {FPDO, fc[best].type, best + 1, 12000, 12000, min_u16(fc[best].max_ma, c->max_ma), 0};
            insert_sorted(fixed, &nf, &e);
        }
    }

    /* 5. 输出 */
    for (uint8_t k = 0; k < nf; k++)
    {
        /* PDO1 只保留 Unconstrained Power 标志，其余角色/能力位全部为 0 */
        uint32_t flags = (k == 0 && fe_caps_unconstrained()) ? (1u << 27) : 0;
        add_entry(&fixed[k], pd_build_fixed_pdo(fixed[k].mv, fixed[k].ma, flags));
    }
    if (has_avs)
        add_entry(&avs, pd_build_spr_avs_apdo(avs.ma, avs.ma20));
    for (uint8_t k = 0; k < np; k++)
        add_entry(&pps[k], pd_build_pps_apdo(pps[k].min_mv, pps[k].mv, pps[k].ma));
}

/* 能力表：PDO[n] 后接每个 PDO 对应的充电器位置 src[n] */
static uint8_t back_caps_blob(uint8_t *out)
{
    memcpy(out, back_raw, back_n * 4);
    for (uint8_t i = 0; i < back_n; i++)
        out[back_n * 4 + i] = entries[i].fe_pos;
    return back_n * 5;
}

static void rebuild_back_caps(void)
{
    build_back_caps();
    uint8_t blob[PD_MAX_DATA_OBJS * 5];
    uint8_t len = back_caps_blob(blob);
    if (len != logged_len || memcmp(blob, logged, len) != 0)
    {
        memcpy(logged, blob, len);
        logged_len = len;
        evlog_add(EV_BE_CAPS, blob, len);
    }
}

/* 后端某个能力项、某个电压对应的前端请求 */
static fe_target_t front_target(const back_entry_t *e, uint16_t mv, uint16_t op_ma)
{
    uint8_t n;
    const pdo_t *c = fe_caps(&n);
    fe_target_t t = {e->fe_pos, e->fe_type, mv, e->ma};
    if (fe_is_legacy() || e->fe_pos == 0 || e->fe_pos > n)
        return t;
    const pdo_t *src = &c[e->fe_pos - 1];
    if (e->fe_type == FPDO)
        t.ma = src->max_ma;
    else if (e->fe_type == PPS_PDO)
        t.ma = (e->type == PPS_PDO) ? op_ma : src->max_ma;     /* PPS 透传按设备限流值，其余用 PPS 最大电流 */
    else
        t.ma = (mv > 15000 && src->max_ma_20v) ? src->max_ma_20v : src->max_ma;
    return t;
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

/* 设备当前合约是否仍在能力项 e 内 */
static bool entry_covers(const back_entry_t *e)
{
    if (e->type != back_type)
        return false;
    if (back_type == FPDO)
        return e->mv == back_mv;
    if (back_type == PPS_PDO)
        return back_mv >= e->min_mv && back_mv <= e->mv && back_ma <= e->ma;
    return back_mv >= 9000 && back_mv <= e->mv;
}

/*
 * 前端能力或设置变化后重建后端能力：能保持设备当前合约就保持（返回前端应请求的目标），
 * 保持不了就回 5V 并让后端 Hard Reset
 */
static fe_target_t reevaluate(void)
{
    uint32_t old_raw[PD_MAX_DATA_OBJS];
    uint8_t old_n = back_n;
    memcpy(old_raw, back_raw, sizeof(old_raw));

    rebuild_back_caps();
    if (old_n != back_n || memcmp(old_raw, back_raw, back_n * 4) != 0)
        back_caps_dirty = true;

    if (!back_has_contract || fe_is_legacy())
        return front_5v_target();

    for (uint8_t i = 0; i < back_n; i++)
    {
        if (entry_covers(&entries[i]))
            return front_target(&entries[i], back_mv, back_ma);
    }

    back_hard_reset = true;
    back_has_contract = false;
    return front_5v_target();
}

/*
 * 转换进行中（设备已收到 Accept、在等 PS_RDY）时前端收到能力报文（充电器 Soft Reset 后重新广播等）：
 * 直接用设备的新目标回应。实测先恢复旧电压、再主动发起新请求，充电器会再次 Soft Reset；
 * 回应能力报文则每次都被接受。能力表变化或超出时限时仍保持当前合约，由转换流程判失败
 */
fe_target_t bridge_on_front_caps(bool *for_bridge)
{
    fe_target_t t = reevaluate();
    *for_bridge = false;
    if ((tr_state == TR_FRONT_REQ || tr_state == TR_FRONT_WAIT) && !back_caps_dirty && !back_hard_reset &&
        millis() - tr_start_ts < T_RETRY_BUDGET_MS && map_back_request(tr_rdo))
    {
        tr_state = TR_FRONT_WAIT;
        tr_front_sent = true;
        tr_ts = millis();
        *for_bridge = true;
        return tr_target;
    }
    return t;
}

void bridge_on_cfg_changed(void)
{
    evlog_add(EV_CFG, cfg(), sizeof(cfg_t));
    if (fe_caps_available())
        reevaluate();
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
    return c && c->type == FPDO && c->mv == 5000 && tr_state == TR_IDLE && !want_front_5v && !front_5v_pending;
}

uint8_t bridge_back_caps(uint32_t *pdos)
{
    if (back_n == 0)
        rebuild_back_caps();
    memcpy(pdos, back_raw, back_n * 4);
    back_caps_dirty = false;
    return back_n;
}

uint8_t bridge_back_caps_peek(uint8_t *blob)
{
    back_caps_blob(blob);
    return back_n;
}

static void log_request(uint8_t result)
{
    if (result == REQ_OK && tr_back_type == PPS_PDO && !(cfg()->flags & CFG_LOG_PPS))
        return;     /* PPS 会频繁调压，默认不记录成功的 PPS 请求 */
    if (result == REQ_OK)
    {
        if (tr_rdo == back_rdo_logged)
            return;     /* 重复请求（PPS 保活、重新协商同一档） */
        back_rdo_logged = tr_rdo;
    }
    ev_request_t r = {
        .back_rdo = tr_rdo,
        .front_rdo = fe_contract_rdo(),
        .mv = tr_mv,
        .vbus_mv = analog_vbus_mv(),
        .result = result,
        .repeat = 0,
        .ms = result == REQ_REJECT ? 0 : (uint16_t)(millis() - tr_start_ts),
        .waits = (result != REQ_REJECT && tr_front_sent) ? fe_request_waits() : 0,
        .txwait = result != REQ_REJECT ? tr_txwait : 0,
    };
    evlog_request(&r, tr_back_type != FPDO);
}

/* 把设备 RDO 映射为前端目标（写入 tr_*）；超出能力返回 false */
static bool map_back_request(uint32_t rdo)
{
    uint8_t pos = pd_rdo_pos(rdo);
    if (pos == 0 || pos > back_n)
        return false;

    const back_entry_t *e = &entries[pos - 1];
    uint16_t mv, op_ma;
    switch (e->type)
    {
    case FPDO:
        op_ma = pd_rdo_fixed_op_ma(rdo);
        if (op_ma > e->ma)
            return false;
        mv = e->mv;
        break;

    case PPS_PDO:
        /* PPS：电压 20mV 单位，电流为限流值，原样转给充电器 */
        mv = pd_rdo_pps_mv(rdo);
        op_ma = pd_rdo_avs_ma(rdo);
        if (mv < e->min_mv || mv > e->mv || op_ma > e->ma || op_ma == 0)
            return false;
        if (e->fe_type == SPR_AVS_PDO)
        {
            /* Lab AVS 转 PPS：AVS 100mV 步进，四舍五入；AVS 没有 9V 以下，拒绝 */
            mv = (mv + 50) / 100 * 100;
            if (mv < 9000)
                return false;
        }
        break;

    case SPR_AVS_PDO:
    {
        /* SPR AVS：电压 25mV 单位、100mV 步进；按 9V~AVS 最高电压校验 */
        mv = pd_rdo_avs_mv(rdo) / 100 * 100;
        op_ma = pd_rdo_avs_ma(rdo);
        if (mv < 9000 || mv > e->mv)
            return false;
        uint16_t limit = (mv < 15000) ? e->ma : (mv > 15000) ? e->ma20 : (e->ma > e->ma20 ? e->ma : e->ma20);
        if (op_ma > limit)
            return false;
        break;
    }

    default:
        return false;
    }

    tr_back_type = e->type;
    tr_mv = mv;
    tr_op_ma = op_ma;
    tr_target = front_target(e, mv, op_ma);
    return true;
}

bool bridge_eval_back_request(uint32_t rdo)
{
    tr_rdo = rdo;
    tr_mv = 0;
    if (map_back_request(rdo))
        return true;
    log_request(REQ_REJECT);
    return false;
}

void bridge_start_transition(void)
{
    const fe_target_t *c = fe_contract();
    tr_ts = millis();
    tr_start_ts = tr_ts;
    tr_front_sent = false;
    tr_txwait = 0;
    if (fe_is_legacy() ||
        (c && c->type == tr_target.type && c->pos == tr_target.pos && c->mv == tr_target.mv &&
         c->ma == tr_target.ma))
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
            tr_front_sent = true;
            tr_state = TR_FRONT_WAIT;
            tr_ts = now;
        }
        else if (now - tr_ts > T_FRONT_REQ_START_MS)
        {
            tr_state = TR_IDLE;
            log_request(REQ_FRONT_FAIL);
            return BRIDGE_FAIL;
        }
        return BRIDGE_PENDING;

    case TR_FRONT_WAIT:
    {
        fe_req_status_t s = fe_request_status();
        uint8_t w = fe_request_txwait();
        if (w > tr_txwait)
            tr_txwait = w;
        if (s == FE_REQ_OK)
        {
            tr_state = TR_VERIFY;
            tr_ts = now;
            tr_stable = 0;
            tr_sample_ts = 0;
        }
        else if (s == FE_REQ_ABORTED && now - tr_start_ts < T_RETRY_BUDGET_MS && map_back_request(tr_rdo))
        {
            /* 充电器 Soft Reset 或重新广播：电压未变，前端恢复后按新能力表重发 */
            tr_state = TR_FRONT_REQ;
            tr_ts = now;
        }
        else if (s == FE_REQ_FAIL || s == FE_REQ_ABORTED || now - tr_ts > T_FRONT_DONE_MS)
        {
            tr_state = TR_IDLE;
            log_request(REQ_FRONT_FAIL);
            return BRIDGE_FAIL;
        }
        return BRIDGE_PENDING;
    }

    case TR_VERIFY:
        if (now - tr_sample_ts >= VBUS_SAMPLE_INTERVAL_MS)
        {
            tr_sample_ts = now;
            uint16_t v = analog_vbus_mv();
            bool ok = !vbus_over(v, tr_mv) && !vbus_under(v, tr_mv);
            tr_stable = ok ? tr_stable + 1 : 0;
            if (tr_stable >= VBUS_STABLE_SAMPLES)
            {
                tr_state = TR_IDLE;
                log_request(REQ_OK);
                return BRIDGE_OK;
            }
        }
        if (now - tr_ts > T_VERIFY_MS)
        {
            tr_state = TR_IDLE;
            log_request(REQ_VERIFY_FAIL);
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
    back_ma = tr_op_ma;
    back_rdo = tr_rdo;
    vbus_bad_count = 0;
    ocp_active = false;
}

bool bridge_rear_query_wanted(void)
{
    return avs_held && !rear_queried;
}

void bridge_on_rear_sink_modes(int16_t modes)
{
    rear_queried = true;
    ev_avs_2nd_t e = {1, modes < 0 ? AVS2_BE_NO_SKEDB : AVS2_BE_SKEDB_RCVD, modes < 0 ? 0 : (uint8_t)modes};
    evlog_add(EV_AVS_2ND, &e, sizeof(e));
    if (modes >= 0 && (modes & SINK_MODE_AVS))
    {
        rear_avs_ok = true;
        reevaluate();       /* 重新广播，加入 AVS */
    }
}

uint8_t bridge_rear_avs_2nd_state(void)
{
    if (rear_avs_ok)
        return 3;
    if (!avs_held)
        return 0;
    return rear_queried ? 2 : 1;
}

/* 设备拔出：二次握手从头开始。Hard Reset 不重置，否则 PPS 合约 → 改给 AVS → Hard Reset 会循环 */
void bridge_on_back_detach(void)
{
    rear_queried = false;
    if (rear_avs_ok)
    {
        rear_avs_ok = false;
        rebuild_back_caps();
    }
}

void bridge_on_back_reset(void)
{

    back_has_contract = false;
    back_hard_reset = false;
    back_rdo_logged = 0;
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

bool bridge_back_contract(uint32_t *rdo, uint16_t *mv)
{
    *rdo = back_has_contract ? back_rdo : 0;
    *mv = back_has_contract ? back_mv : 0;
    return back_has_contract;
}

bool bridge_flash_safe(void)
{
    return tr_state == TR_IDLE && !want_front_5v && !front_5v_pending;
}

static void protection_trip(uint8_t kind, uint16_t v, uint16_t i)
{
    power_sw_set(false);
    ev_protect_t e = {kind, v, i, back_mv};
    evlog_add(EV_PROTECT, &e, sizeof(e));
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
            {
                want_front_5v = false;
                front_5v_pending = true;
            }
        }
    }
    if (front_5v_pending)
    {
        fe_req_status_t s = fe_request_status();
        if (s == FE_REQ_OK || s == FE_REQ_FAIL)
        {
            ev_fe_contract_t e = {fe_contract_rdo(), 5000, s == FE_REQ_OK};
            evlog_add(EV_FE_CONTRACT, &e, sizeof(e));
            front_5v_pending = false;
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

    const cfg_t *c = cfg();

    /* 过/欠压：连续 3 次超出窗口；PPS 合约下充电器可能处于限流模式，只做过压 */
    uint16_t v = analog_vbus_mv();
    uint8_t kind = vbus_over(v, back_mv) ? PROT_OVP
                 : (back_type != PPS_PDO && vbus_under(v, back_mv)) ? PROT_UVP : 0;
    if (kind)
    {
        vbus_bad_kind = kind;
        if (++vbus_bad_count >= VBUS_STABLE_SAMPLES)
        {
            protection_trip(vbus_bad_kind, v, analog_current_ma());
            return;
        }
    }
    else
    {
        vbus_bad_count = 0;
    }

    /* 过流：超过设定电流持续设定时间 */
    uint16_t i = analog_current_ma();
    if (i > c->ocp_ma)
    {
        if (!ocp_active)
        {
            ocp_active = true;
            ocp_ts = now;
        }
        else if (now - ocp_ts >= c->ocp_ms)
        {
            protection_trip(PROT_OCP, v, i);
        }
    }
    else
    {
        ocp_active = false;
    }
}
