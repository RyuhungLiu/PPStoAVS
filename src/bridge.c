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
#define COMP_MAX_MV             1000    /* 压降补偿上限 */
#define COMP_ABS_MAX_MV         21000   /* 补偿后前端电压绝对上限（SPR） */
#define T_COMP_UPDATE_MS        500     /* R 补偿：按电流更新前端电压的周期 */
#define T_COMP_FAIL_MS          5000    /* 补偿请求被拒后暂停 */

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
static bool back_cable_5a;      /* 后端线材 E-Marker 读到 5A（be_source 读取） */

/* Lab：后端 AVS 二次握手（CFG_BE_AVS_2ND）——先给 PPS，设备在 Sink_Capabilities_Extended 中声明支持 AVS 后才给 AVS */
static bool avs_held;           /* 本可提供 AVS，但尚未确认设备支持 */
static bool rear_queried;       /* 本次连接已查询过设备 */
static bool rear_avs_ok;        /* 设备已声明支持 AVS */

/* 第二组自订 PDO 生效条件：强制 PPS + 自订 PDO 档位 + CFGX_PDO2，且第二组非空；此时 CFG_BE_AVS_2ND 不起作用 */
static bool pdo2_mode(void)
{
    const cfg_ext_t *x = cfg_ext();
    return (cfg()->flags2 & CFG2_FORCE_PPS) && (x->flags & CFGX_PDO) && (x->flags & CFGX_PDO2) && cfg_pdo2()->n > 0;
}

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

/* 压降补偿 */
static uint32_t comp_ts;
static bool comp_pending;

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

static uint16_t max_u16(uint16_t a, uint16_t b)
{
    return a > b ? a : b;
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

/* 前端 EPR AVS：覆盖 15~20V */
static int8_t find_epr_avs(const pdo_t *c, uint8_t n)
{
    if (!(cfg()->flags & CFG_EPR_AVS) || !fe_epr_mode())
        return -1;
    for (uint8_t i = 0; i < n; i++)
    {
        if (c[i].type == EPR_AVS_PDO && c[i].min_mv <= 15000 && c[i].max_mv >= 20000 && c[i].pdp > 0)
            return i;
    }
    return -1;
}

/* 充电器某电压 Fixed 的位置（1-based），没有返回 0 */
static uint8_t find_front_fixed(uint16_t mv)
{
    uint8_t n;
    const pdo_t *c = fe_caps(&n);
    for (uint8_t i = 0; i < n; i++)
    {
        if (c[i].type == FPDO && c[i].raw != 0 && c[i].max_mv == mv)
            return i + 1;
    }
    return 0;
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
 *  - Lab 自订 Fixed（CFG_FIX_CUSTOM，与 12V 转换互斥）：任意 5.1~20V（100mV）与电流，来源同 12V 转换，
 *    电流取 min(设定, 来源, 电流上限)；充电器已有同电压 Fixed 时不转换
 *  - 个数 ≤ 7，优先级 Fixed > AVS > PPS（保留最高电压高的）> 12V 转换
 *  - 顺序：Fixed（电压升序）、AVS、PPS（最高电压升序）
 *  - Lab 后端二次握手：确认设备支持 AVS 之前不给 AVS、改给全部 PPS
 *  - Lab EPR AVS（CFG_EPR_AVS，前端 EPR 模式）：AVS 来源优先取 EPR AVS，只用 15~20V（电流 min(PDP÷20V, 5A)）；
 *    9~15V 段空缺，只有 9V、15V 两点走 Fixed，9~15V 电流 = min(9V、15V Fixed 电流)
 *  - Lab 自订 PDO 列表（CFGX_PDO，需强制 PPS）：列表里的 Fixed / PPS / AVS 勾选取代上面的来源规则（模式 a~d 仍过滤，隐藏 Fixed、12V 转换、自订 Fixed / PPS 忽略）；
 *    Fixed 取覆盖该电压的 PPS → 充电器 Fixed → 9V 起 AVS，PPS 取完整覆盖的 PPS → AVS，电流取 min(设定, 来源, 电流上限)；AVS 沿用上面的来源与电流规则
 *  - Lab AVS 转 PPS（CFG_AVS_TO_PPS，模式 b/c）：充电器原生 AVS 另外提供为 9V（默认）或 5V（CFG2_AVS_PPS5）~AVS 最高电压的 PPS，
 *    电流取两段 AVS 电流较小者；同一最高电压已有充电器 PPS 时以充电器 PPS 为准
 */
/* 覆盖某电压、电流最大的充电器 PPS（下标），没有返回 -1 */
static int8_t find_pps_covering(const pdo_t *fc, uint8_t n, uint16_t mv)
{
    int8_t best = -1;
    for (uint8_t i = 0; i < n; i++)
    {
        if (fc[i].type == PPS_PDO && fc[i].min_mv <= mv && fc[i].max_mv >= mv &&
            (best < 0 || fc[i].max_ma > fc[best].max_ma))
            best = i;
    }
    return best;
}

/* lo~hi 的 PPS：由充电器 PPS 或原生 AVS（9V 起）提供。来源达不到整个范围时不放弃，而是把电压范围缩到来源能给的部分：
 * 取与 lo~hi 交集最宽的来源（同宽时 PPS 优先，再取电流大者），电流取 min(来源, ma)（请求按 100mV 四舍五入） */
static bool pps_from_source(const pdo_t *fc, uint8_t n, uint16_t lo, uint16_t hi, uint16_t ma, back_entry_t *out)
{
    int8_t best = -1;
    uint16_t blo = 0, bhi = 0, bma = 0;
    for (uint8_t i = 0; i < n && lo < hi; i++)
    {
        if (fc[i].type != PPS_PDO)
            continue;
        uint16_t olo = max_u16(lo, fc[i].min_mv), ohi = min_u16(hi, fc[i].max_mv);
        if (olo >= ohi || fc[i].max_ma == 0)
            continue;
        if (best < 0 || ohi - olo > bhi - blo || (ohi - olo == bhi - blo && fc[i].max_ma > bma))
        {
            best = i;
            blo = olo;
            bhi = ohi;
            bma = fc[i].max_ma;
        }
    }
    int8_t a = find_native_avs(fc, n);
    if (a >= 0 && lo < hi)
    {
        uint16_t olo = max_u16(lo, 9000), ohi = min_u16(hi, fc[a].max_mv);
        uint16_t ama = (ohi > 15000 && fc[a].max_ma_20v) ? min_u16(fc[a].max_ma, fc[a].max_ma_20v) : fc[a].max_ma;
        if (olo < ohi && ama > 0 && (best < 0 || ohi - olo > bhi - blo))
        {
            best = a;
            blo = olo;
            bhi = ohi;
            bma = ama;
        }
    }
    if (best < 0)
        return false;
    back_entry_t e = {PPS_PDO, fc[best].type, best + 1, blo, bhi, min_u16(bma, ma), 0};
    *out = e;
    return true;
}

/* 自订 PDO 列表里的 Fixed：强制 PPS，覆盖该电压的 PPS 优先，其次充电器同电压 Fixed，最后 9V 起的原生 AVS；电流取 min(设定, 来源) */
static bool list_fixed(const pdo_t *fc, uint8_t n, const pdo_t *p, uint16_t max_ma, back_entry_t *out)
{
    uint16_t mv = p->max_mv;
    int8_t s = find_pps_covering(fc, n, mv);
    if (s < 0 && find_front_fixed(mv))
        s = find_front_fixed(mv) - 1;
    uint16_t src_ma = s >= 0 ? fc[s].max_ma : 0;
    if (s < 0)
    {
        s = find_native_avs(fc, n);
        if (s < 0 || mv < 9000 || mv > fc[s].max_mv)
            return false;
        src_ma = (mv > 15000 && fc[s].max_ma_20v) ? fc[s].max_ma_20v : fc[s].max_ma;
    }
    if (src_ma == 0)
        return false;
    back_entry_t e = {FPDO, fc[s].type, s + 1, mv, mv, min_u16(min_u16(p->max_ma, max_ma), src_ma), 0};
    *out = e;
    return true;
}

static void build_back_caps(void)
{
    cfg_t eff = *cfg();
    eff.max_ma = bridge_back_max_ma();      /* 线材 5A 时放宽默认的 3A 上限（Source 不支持 EPR，只到 20V） */
    const cfg_t *c = &eff;
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

    /* Lab 自订 PDO 档位（强制 PPS 下）：Fixed / AVS / PPS 都取自列表，仍按模式 a~d 过滤；AVS 须有 15V Fixed，列表里同时有 20V Fixed 才到 20V，否则 AVS 只到 15V（设置里已校验） */
    bool force_pps = (c->flags2 & CFG2_FORCE_PPS) != 0;
    const cfg_ext_t *x = cfg_ext();
    bool list = force_pps && (x->flags & CFGX_PDO);
    /* 第二组自订 PDO：设备声明支持 AVS 之后改用（见 pdo2_mode） */
    bool second = rear_avs_ok && pdo2_mode();
    pdo_t lp[CFGX_MAX_PDOS];
    uint8_t nl = list ? (second ? cfg_pdo2()->n : x->pdo_n) : 0;
    for (uint8_t i = 0; i < nl; i++)
        lp[i] = pd_parse_pdo(second ? cfg_pdo2()->pdo[i] : x->pdo[i]);
    if (list)
    {
        bool avs_item = false;
        for (uint8_t i = 0; i < nl; i++)
            avs_item |= lp[i].type == SPR_AVS_PDO;
        want_avs &= avs_item;
    }

    /* 1. 充电器 Fixed */
    back_entry_t fixed[PD_MAX_DATA_OBJS];
    uint8_t nf = 0;
    for (uint8_t i = 0; i < nl; i++)
    {
        back_entry_t e;
        if (lp[i].type == FPDO && lp[i].max_mv <= c->max_mv && list_fixed(fc, n, &lp[i], c->max_ma, &e))
            insert_sorted(fixed, &nf, &e);
    }
    for (uint8_t i = 0; i < n && !list; i++)
    {
        uint16_t mv = fc[i].max_mv;
        if (fc[i].type != FPDO || fc[i].raw == 0 || mv > c->max_mv || fixed_hidden(mv))
            continue;   /* raw 0：EPR 能力表中 SPR 位置的空位 */
        back_entry_t e = {FPDO, FPDO, i + 1, mv, mv, min_u16(fc[i].max_ma, c->max_ma), 0};
        insert_sorted(fixed, &nf, &e);
    }

    /*
     * 1b. Lab 强制 PPS：Fixed 改由覆盖该电压的 PPS 提供（电流大者优先），电流按 PPS。
     * 前端合约因此都是 PPS，压降补偿对所有档位生效；没有 PPS 覆盖的电压仍走充电器 Fixed
     */
    if (force_pps && !list)
    {
        for (uint8_t k = 0; k < nf; k++)
        {
            int8_t p = find_pps_covering(fc, n, fixed[k].mv);
            if (p < 0)
                continue;
            fixed[k].fe_type = PPS_PDO;
            fixed[k].fe_pos = p + 1;
            fixed[k].ma = min_u16(fc[p].max_ma, c->max_ma);
        }
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
        int8_t epr = find_epr_avs(fc, n);
        int8_t f9 = find_mv(fixed, nf, 9000);
        if (epr >= 0 && c->max_mv >= 20000 && find_mv(fixed, nf, 20000) >= 0)
        {
            src = &fc[epr];
            src_pos = epr + 1;
            s15 = f9 >= 0 ? fixed[f9].ma : fixed[f15].ma;     /* 9~15V 只有 9V、15V 两点（Fixed） */
            s20 = pd_epr_avs_ma(src, 20000);
        }
        else if (nat_tier > 0 && (force_pps ? nat_tier > pps_tier : nat_tier >= pps_tier))   /* 强制 PPS：同等范围时用 PPS */
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
            bool from_epr = src->type == EPR_AVS_PDO;
            bool drop20 = avs_max < 20000 && f20 >= 0;
            if (nf - drop20 < PD_MAX_DATA_OBJS && (c->flags & CFG_BE_AVS_2ND) && !pdo2_mode() && !rear_avs_ok)
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
                uint16_t ma20 = 0;
                if (from_epr)
                {
                    /* 15~20V 由 EPR AVS 提供，Fixed 电流不变；9~15V 电流不超过 9V/15V Fixed */
                    ma20 = min_u16(s20, c->max_ma);
                }
                else
                {
                    fixed[f15].ma = ma15;
                    if (avs_max == 20000)
                    {
                        ma20 = min_u16(fixed[f20].ma, s20);
                        fixed[f20].ma = ma20;
                    }
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
    for (uint8_t i = 0; i < nl && want_pps; i++)
    {
        back_entry_t e;
        if (lp[i].type == PPS_PDO && lp[i].min_mv < c->max_mv &&
            pps_from_source(fc, n, lp[i].min_mv, min_u16(lp[i].max_mv, c->max_mv), min_u16(lp[i].max_ma, c->max_ma), &e))
            insert_sorted(pps, &np, &e);
    }
    if ((want_pps || avs_held) && !list)
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
            uint16_t lo = (c->flags2 & CFG2_AVS_PPS5) ? 5000 : 9000;
            back_entry_t e = {PPS_PDO, SPR_AVS_PDO, nat + 1, lo, mv, min_u16(ma, c->max_ma), 0};
            insert_sorted(pps, &np, &e);    /* 同一最高电压已有充电器 PPS 则不插入 */
        }
    }

    /*
     * 3b. Lab 自订 PPS：由完整覆盖该电压范围、电流最大的充电器 PPS 提供；没有时 9V 起的范围可由 SPR AVS 提供
     * （请求按 100mV 四舍五入）。最高电压不超过最高电压设置；各模式都提供，位置优先于 PPS 透传与转换 Fixed
     */
    bool has_custom = false;
    back_entry_t custom;
    if ((c->flags2 & CFG2_PPS_CUSTOM) && !list)
    {
        uint16_t lo = c->pps_min_dv * 100, hi = min_u16(c->pps_max_dv * 100, c->max_mv);
        if (pps_from_source(fc, n, lo, hi, min_u16(c->pps_ma50 * 50, c->max_ma), &custom))
        {
            has_custom = true;
            int8_t k = find_mv(pps, np, custom.mv);
            if (k >= 0)
            {
                for (; k + 1 < np; k++)     /* 同一最高电压的透传 PPS 让位 */
                    pps[k] = pps[k + 1];
                np--;
            }
        }
    }
    uint8_t room = PD_MAX_DATA_OBJS - nf - has_avs - has_custom;
    if (np > room)
    {
        /* 位置不够：保留最高电压高的 */
        for (uint8_t k = 0; k < room; k++)
            pps[k] = pps[np - room + k];
        np = room;
    }
    if (has_custom)
        insert_sorted(pps, &np, &custom);

    /* 4. 转换 Fixed：12V 转换或 Lab 自订 Fixed（互斥） */
    uint16_t conv_mv = 0, conv_ma = c->max_ma;
    if (list)
    {
        /* 自订 PDO 列表已包含全部 Fixed，不再转换 */
    }
    else if ((c->flags & CFG_FIX12) && !fixed_hidden(12000))
    {
        conv_mv = 12000;
    }
    else if (c->flags & CFG_FIX_CUSTOM)
    {
        conv_mv = c->fix_dv * 100;
        conv_ma = min_u16(conv_ma, c->fix_ma50 * 50);
    }
    if (conv_mv && conv_mv <= c->max_mv && find_mv(fixed, nf, conv_mv) < 0 && nf + has_avs + np < PD_MAX_DATA_OBJS)
    {
        int8_t best = find_pps_covering(fc, n, conv_mv);
        uint16_t src_ma = best >= 0 ? fc[best].max_ma : 0;
        if (best < 0)
        {
            int8_t a = find_native_avs(fc, n);
            if (a >= 0 && conv_mv >= 9000 && conv_mv <= fc[a].max_mv)
            {
                best = a;
                src_ma = conv_mv > 15000 ? fc[a].max_ma_20v : fc[a].max_ma;
            }
        }
        if (best >= 0 && src_ma > 0)
        {
            back_entry_t e = {FPDO, fc[best].type, best + 1, conv_mv, conv_mv, min_u16(src_ma, conv_ma), 0};
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

static bool adjustable(pdo_type_t t)
{
    return t == PPS_PDO || t == SPR_AVS_PDO || t == EPR_AVS_PDO;
}

/*
 * 压降补偿（前端为 PPS/AVS 时，把请求电压上调以抵消转换器与线材压降；Fixed 无法补偿）：
 *  - V 补偿：固定加 comp_val mV
 *  - R 补偿：加 I × comp_val（mΩ），I 为输出电流
 * 上限 1V，按前端步进（PPS 20mV、AVS 100mV）四舍五入，不超过来源 PDO 最高电压
 */
static uint16_t comp_mv(uint16_t ma)
{
    const cfg_t *c = cfg();
    uint32_t v = c->comp_mode == CFG_COMP_V ? c->comp_val
               : c->comp_mode == CFG_COMP_R ? (uint32_t)ma * c->comp_val / 1000 : 0;
    return v > COMP_MAX_MV ? COMP_MAX_MV : v;
}

static uint16_t comp_step(pdo_type_t t)
{
    return t == PPS_PDO ? 20 : 100;
}

static void apply_comp(fe_target_t *t, uint16_t comp)
{
    uint8_t n;
    const pdo_t *c = fe_caps(&n);
    if (!adjustable(t->type) || comp == 0 || t->pos == 0 || t->pos > n)
        return;
    uint16_t step = comp_step(t->type);
    uint16_t max = min_u16(c[t->pos - 1].max_mv, COMP_ABS_MAX_MV) / step * step;
    uint32_t mv = ((uint32_t)t->mv + comp + step / 2) / step * step;
    if (mv > max)
        mv = max;
    if (mv > t->mv)
        t->mv = mv;
}

/* 前端 VBUS 的期望值：可调来源为实际请求电压（含补偿），其余为设备电压 */
static uint16_t front_expect_mv(uint16_t back_target)
{
    const fe_target_t *c = fe_contract();
    if (c && adjustable(c->type) && c->mv > back_target && c->mv - back_target <= COMP_MAX_MV + 100)
        return c->mv;
    return back_target;
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
    else if (e->fe_type == EPR_AVS_PDO)
    {
        /* EPR AVS 只有 15V 以上：9V、15V 走充电器 Fixed */
        uint8_t fpos = mv <= 15000 ? find_front_fixed(mv) : 0;
        if (fpos)
        {
            t.pos = fpos;
            t.type = FPDO;
            t.ma = c[fpos - 1].max_ma;
        }
        else
        {
            t.ma = pd_epr_avs_ma(src, mv);
        }
    }
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
    /* PPS 不比电流：能力表电流变小（如 QC 合成 PPS 跨档）时保持合约，重新广播后设备会按新电流重新请求 */
    if (back_type == PPS_PDO)
        return back_mv >= e->min_mv && back_mv <= e->mv;
    return back_mv >= 9000 && back_mv <= e->mv;
}

/*
 * 设备当前合约对应的能力项：先按 RDO 中的位置找（模式 b/c 可能有多个 PPS 覆盖同一电压，
 * 如 5~11V 与 5~20V，按电压找会找到别的充电器 PPS）；能力表重建后位置对不上才按电压找
 */
static const back_entry_t *contract_entry(void)
{
    uint8_t pos = pd_rdo_pos(back_rdo);
    if (pos >= 1 && pos <= back_n && entry_covers(&entries[pos - 1]))
        return &entries[pos - 1];
    for (uint8_t i = 0; i < back_n; i++)
    {
        if (entry_covers(&entries[i]))
            return &entries[i];
    }
    return NULL;
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

    const back_entry_t *e = contract_entry();
    if (e)
    {
        fe_target_t t = front_target(e, back_mv, back_ma);
        apply_comp(&t, comp_mv(analog_current_ma()));
        return t;
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

/*
 * 后端电流上限：设置里的上限；默认 3A 视为“自动”，后端线材的 E-Marker 读到 5A 时放宽到 5A
 * （充电器自己给的各档电流仍是上限，所以 5A 只会出现在充电器提供 5A 的档位，通常是 20V）。
 * 虚拟 E-Marker（CFGX_CABLE）开启时以它的电流位为准（3A 即不放宽）；用户改过上限（≠ 3A）时按设置
 */
uint16_t bridge_back_max_ma(void)
{
    const cfg_t *c = cfg();
    const cfg_ext_t *x = cfg_ext();
    bool five = (x->flags & CFGX_CABLE) ? cfg_ext_cable_5a(x) : back_cable_5a;
    if (five && c->max_ma == CFG_MAX_MA)
        return CFG_MAX_MA_5A;
    return c->max_ma;
}

/* 过流：后端进入 5A（E-Marker 5A 且上限放宽到 5A）时固定 5350mA / 50ms，否则按设置 */
#define OCP_5A_MA 5350
#define OCP_5A_MS 50
static bool ocp_is_5a(void)
{
    return bridge_back_max_ma() >= CFG_MAX_MA_5A;
}

static uint16_t ocp_limit_ma(void)
{
    return ocp_is_5a() ? OCP_5A_MA : cfg()->ocp_ma;
}

static uint16_t ocp_time_ms(void)
{
    return ocp_is_5a() ? OCP_5A_MS : cfg()->ocp_ms;
}

/* 后端线材 E-Marker 的电流能力变化（5A 与否）：重建能力并重新广播 */
void bridge_on_back_cable(bool five_amp)
{
    if (back_cable_5a == five_amp)
        return;
    back_cable_5a = five_amp;
    if (fe_caps_available())
        reevaluate();
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
        if (e->fe_type == EPR_AVS_PDO && mv <= 15000)
        {
            /* EPR AVS 只有 15~20V：9V、15V 走充电器 Fixed（front_target），其余 9~15V 拒绝 */
            if ((mv != 9000 && mv != 15000) || find_front_fixed(mv) == 0)
                return false;
            limit = e->ma;
        }
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
    apply_comp(&tr_target, comp_mv(analog_current_ma()));
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
            uint16_t expect = front_expect_mv(tr_mv);
            bool ok = !vbus_over(v, expect) && !vbus_under(v, expect);
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
    return (avs_held || pdo2_mode()) && !rear_queried;
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
    if (!avs_held && !pdo2_mode())
        return 0;
    return rear_queried ? 2 : 1;
}

/* 设备拔出：二次握手从头开始。Hard Reset 不重置，否则 PPS 合约 → 改给 AVS → Hard Reset 会循环 */
void bridge_on_back_detach(void)
{
    rear_queried = false;
    if (rear_avs_ok || back_cable_5a)
    {
        rear_avs_ok = false;
        back_cable_5a = false;
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

/* 记录补偿量（前端请求 − 设备电压），变化 ≥100mV 才记，避免 R 补偿随电流波动刷屏 */
static void log_comp(void)
{
    static int16_t logged = -1;
    const fe_target_t *c = fe_contract();
    if (!c || c->mv < back_mv)
        return;
    int16_t d = (c->mv - back_mv) / 10;
    if (logged >= 0 && d > logged - 10 && d < logged + 10)
        return;
    logged = d;
    ev_note_t e = {0, NOTE_COMP, d > 255 ? 255 : (uint8_t)d};
    evlog_add(EV_NOTE, &e, sizeof(e));
}

/* 按当前电流更新补偿（R 补偿随电流变化；V 补偿在设置改变后生效） */
static void comp_update(uint32_t now, uint16_t ma)
{
    if (comp_pending)
    {
        fe_req_status_t s = fe_request_status();
        if (s == FE_REQ_BUSY)
            return;
        comp_pending = false;
        if (s != FE_REQ_OK)
            comp_ts = now + T_COMP_FAIL_MS;
        else
            log_comp();
    }
    if ((int32_t)(now - comp_ts) < T_COMP_UPDATE_MS || !fe_is_ready() || want_front_5v || front_5v_pending)
        return;
    comp_ts = now;

    const fe_target_t *cur = fe_contract();
    if (!cur || !adjustable(cur->type))
        return;
    const back_entry_t *e = contract_entry();
    if (e)
    {
        fe_target_t t = front_target(e, back_mv, back_ma);
        if (t.type != cur->type || t.pos != cur->pos)
            return;
        /* 迟滞：与现在的请求相差 3/4 步进以上才更新 */
        uint16_t step = comp_step(t.type);
        uint32_t raw = (uint32_t)t.mv + comp_mv(ma);
        int32_t diff = (int32_t)raw - cur->mv;
        if (diff < 0)
            diff = -diff;
        if (diff * 4 < step * 3)
            return;
        apply_comp(&t, comp_mv(ma));
        if (t.mv != cur->mv && fe_request(&t))
            comp_pending = true;
        return;
    }
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


    /* 过/欠压：连续 3 次超出窗口；PPS 合约下充电器可能处于限流模式，只做过压 */
    uint16_t v = analog_vbus_mv();
    uint16_t expect = front_expect_mv(back_mv);
    uint8_t kind = vbus_over(v, expect) ? PROT_OVP
                 : (back_type != PPS_PDO && vbus_under(v, expect)) ? PROT_UVP : 0;
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
    if (i > ocp_limit_ma())
    {
        if (!ocp_active)
        {
            ocp_active = true;
            ocp_ts = now;
        }
        else if (now - ocp_ts >= ocp_time_ms())
        {
            protection_trip(PROT_OCP, v, i);
        }
    }
    else
    {
        ocp_active = false;
    }

    comp_update(now, i);
}
