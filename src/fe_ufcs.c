#include "fe_ufcs.h"
#include "analog.h"
#include "bridge.h"
#include "board.h"
#include "cfg.h"
#include "evlog.h"
#include "power_sw.h"
#include "afc.h"
#include "qc.h"
#include "timebase.h"
#include "ufcs.h"
#include "ufcs_codec.h"

typedef enum
{
    FU_OFF,
    FU_WAIT_CAPS,       /* 会话已建立，等充电器的输出模式 */
    FU_READY,
    FU_REQ,             /* 请求已接受，等引擎发出并等 Power_Ready */
} fu_state_t;

typedef enum
{
    ORIGIN_CAPS,        /* 响应能力（上电、重新广播） */
    ORIGIN_BRIDGE,      /* 协议桥请求 */
} origin_t;

static fu_state_t st;
static pdo_t caps[7];
static uint8_t num_caps;
static uint8_t src_mode[7];             /* 每个合成 PDO 对应的 UFCS 模式编号 */

static bool has_contract;
static fe_target_t contract, pending;
static uint32_t contract_rdo, pending_rdo;
static origin_t pending_origin;
static bool pending_valid;              /* 已接受请求，等引擎空闲后发出 */
static fe_req_status_t req_status = FE_REQ_IDLE;
static uint8_t qcf;                     /* QC 前端：qc_detect 的结果（QC_*），0 = 不是 QC */
static uint8_t afcn, afcl[7];           /* AFC 前端：充电器的 V/I 表（按电压升序、同电压取大电流），afcn = 0 = 不是 AFC */

bool feu_active(void)
{
    return st != FU_OFF;
}

bool feu_start(void)
{
    if (!ufcs_connect(true))
        return false;
    st = FU_WAIT_CAPS;
    has_contract = false;
    pending_valid = false;
    num_caps = 0;
    req_status = FE_REQ_IDLE;
    return true;
}

/* AFC 的 V/I 表整理：校验范围、按电压升序、同电压取大电流；第一个必须是 5V（没有就补 5V 0.75A） */
static uint8_t afc_table(const uint8_t *raw, uint8_t n)
{
    uint8_t k = 0;
    for (uint8_t i = 0; i < n; i++)
    {
        uint8_t b = raw[i], j = 0;
        while (j < k && afc_mv(afcl[j]) < afc_mv(b))
            j++;
        if (j < k && afc_mv(afcl[j]) == afc_mv(b))
        {
            if ((b & 15) > (afcl[j] & 15))
                afcl[j] = b;
            continue;
        }
        if (k >= sizeof(afcl))
            continue;
        for (uint8_t m = k; m > j; m--)
            afcl[m] = afcl[m - 1];
        afcl[j] = b;
        k++;
    }
    if (k && afc_mv(afcl[0]) != 5000)
    {
        if (k >= sizeof(afcl))
            k--;
        for (uint8_t m = k; m > 0; m--)
            afcl[m] = afcl[m - 1];
        afcl[0] = 0x00;
        k++;
    }
    return k;
}

/* QC2.0/3.0 与 AFC 前端（Lab，上电时在 UFCS 之后、USB HID 与 PD 前端之前由 main 调用）：共用 BC1.2→HVDCP 握手；
 * 先试 QC（有 9V / 12V / QC3 才算），否则试 AFC（取 V/I 表）。
 * QC 能力按 18W 惯例合成（QC 不报告电流）：Fixed 5V 3A / 9V 2A / 12V 1.5A（探测到的档位），QC3 另给 PPS 3.6~5.9V 3A、3.6~9V 2A、3.6~12V 1.5A；
 * AFC 能力 = 充电器 V/I 表的各档 Fixed（电流按表） */
bool feu_start_dcp(void)
{
    const cfg_t *c = cfg();
    bool want_qc = (c->flags3 & CFG3_QC) != 0, want_afc = (c->flags3 & CFG3_AFC) != 0;
    /* QC 探测会把前端升到 9V / 12V / 5.6V：期间关断后端（此时后端只有非 PD 的 5V 合约），探测结束回到 5V 再恢复 */
    bool sw = power_sw_is_on();
    power_sw_set(false);
    uint16_t hs = qc_handshake(), dv = 0, base = 0;
    qcf = 0;
    afcn = 0;
    if (want_qc)
    {
        if (hs)
            qcf = qc_detect(&dv, &base);
        ev_qc_step_t e = {QCS_CONNECT, qcf, dv, analog_vbus_mv(), base};
        evlog_add(EV_QC_STEP, &e, sizeof(e));
        if (!(qcf & (QC_9V | QC_12V | QC_3)))
        {
            qcf = 0;
            /* QC 档位试过 D+ 3.3V 等电平，AFC 充电器可能已离开 HVDCP 状态：放开 D± 让充电器复位（D+ < 0.325V），再握手一次 */
            if (hs && want_afc)
            {
                qc_release();
                delay_ms(100);
                hs = qc_handshake();
            }
        }
    }
    if (hs && !qcf && want_afc)
    {
        uint8_t raw[8] = {0};
        uint8_t n = afc_query(raw, sizeof(raw));
        afcn = afc_table(raw, n);
        ev_qc_step_t e = {QCS_AFC, n, (uint16_t)(raw[0] | raw[1] << 8), analog_vbus_mv(), (uint16_t)(raw[2] | raw[3] << 8)};
        evlog_add(EV_QC_STEP, &e, sizeof(e));
    }
    if (!qcf && !afcn)
        qc_release();
    if (sw)
        power_sw_set(true);
    if (!qcf && !afcn)
        return false;
    has_contract = false;
    pending_valid = false;
    req_status = FE_REQ_IDLE;
    st = FU_WAIT_CAPS;
    return true;
}

/* Lab 合成单一 PPS（CFG3_QC_PPS1）：一个 3.6V~最高档的 PPS，电流取当前合约电压所在档（qband）；
 * 合约电压跨档（上行超过 5.9V / 9V，下行回到门限以下 200mV）时重建能力并重新广播，设备按新电流重新请求 */
static uint8_t qband;
static const uint16_t qc_mv[3] = {5000, 9000, 12000}, qc_ma[3] = {3000, 2000, 1500}, qc_top[2] = {5900, 9000};

static bool qc_one_pps(void)
{
    return (qcf & QC_3) && (cfg()->flags3 & CFG3_QC_PPS1);
}

static uint8_t qc_caps(uint32_t *raw)
{
    static const uint8_t need[3] = {QC_OK, QC_9V, QC_12V};
    uint8_t n = 0;
    for (uint8_t i = 0; i < 3; i++)
        if (qcf & need[i])
            raw[n++] = pd_build_fixed_pdo(qc_mv[i], qc_ma[i], 0);
    if (qc_one_pps())
        raw[n++] = pd_build_pps_apdo(3600, (qcf & QC_12V) ? 12000 : 9000, qc_ma[qband]);
    else
        for (uint8_t i = 0; i < 3 && (qcf & QC_3); i++)
            if (qcf & need[i])
                raw[n++] = pd_build_pps_apdo(3600, i ? qc_mv[i] : qc_top[0], qc_ma[i]);
    return n;
}

static uint8_t afc_caps(uint32_t *raw)
{
    for (uint8_t i = 0; i < afcn; i++)
        raw[i] = pd_build_fixed_pdo(afc_mv(afcl[i]), afc_ma(afcl[i]), 0);
    return afcn;
}

/* 合约电压所在档（带 200mV 回差） */
static uint8_t qc_band_of(uint16_t mv)
{
    uint8_t b = qband;
    while (b < 2 && mv > qc_top[b])
        b++;
    while (b > 0 && mv <= qc_top[b - 1] - 200)
        b--;
    return b;
}

void feu_stop(void)
{
    ufcs_close();
    st = FU_OFF;
    has_contract = false;
    num_caps = 0;
}

/* 合成 PDO 位置 → 发给充电器的模式、电压、电流（按模式步进取整，夹在范围内） */
static bool map_target(const fe_target_t *t, uint8_t *mode, uint16_t *mv, uint16_t *ma)
{
    if (t->pos < 1 || t->pos > num_caps)
        return false;
    uint8_t n;
    const ufcs_mode_t *md = ufcs_modes(&n);
    const ufcs_mode_t *m = NULL;
    for (uint8_t i = 0; i < n; i++)
        if (md[i].num == src_mode[t->pos - 1])
            m = &md[i];
    if (!m)
        return false;

    if (!ufcs_map_request(m, t->mv, t->ma, mv, ma))
        return false;
    *mode = m->num;
    return true;
}

static uint32_t build_rdo(const fe_target_t *t)
{
    if (t->type == PPS_PDO)
        return pd_build_pps_rdo(t->pos, t->mv, t->ma);
    return pd_build_fixed_rdo(t->pos, t->ma, t->ma);
}

static void begin_request(const fe_target_t *t, origin_t origin)
{
    pending = *t;
    pending_rdo = build_rdo(t);
    pending_origin = origin;
    pending_valid = true;
    st = FU_REQ;
    if (origin == ORIGIN_BRIDGE)
        req_status = FE_REQ_BUSY;
}

static void request_finished(bool ok)
{
    pending_valid = false;
    if (ok)
    {
        contract = pending;
        contract_rdo = pending_rdo;
        has_contract = true;
    }
    if (pending_origin == ORIGIN_CAPS)
    {
        ev_fe_contract_t e = {pending_rdo, pending.mv, ok};
        evlog_add(EV_FE_CONTRACT, &e, sizeof(e));
    }
    else
    {
        req_status = ok ? FE_REQ_OK : FE_REQ_FAIL;
    }
    st = FU_READY;
}

/* 新的能力：合成 PDO，交给协议桥决定初始请求（与 PD 前端收到 Source_Capabilities 时相同） */
static void on_caps(void)
{
    uint8_t n;
    uint32_t raw[7];
    if (afcn)
        num_caps = afc_caps(raw);
    else if (qcf)
        num_caps = qc_caps(raw);
    else
    {
        const ufcs_mode_t *md = ufcs_modes(&n);
        num_caps = ufcs_synth_caps(md, n, raw, src_mode);
    }
    for (uint8_t i = 0; i < num_caps; i++)
        caps[i] = pd_parse_pdo(raw[i]);
    if (!num_caps)
        return;
    evlog_add(EV_FE_CAPS, raw, num_caps * 4);

    if (req_status == FE_REQ_BUSY)
        req_status = FE_REQ_ABORTED;
    pending_valid = false;
    bool for_bridge;
    fe_target_t t = bridge_on_front_caps(&for_bridge);
    begin_request(&t, for_bridge ? ORIGIN_BRIDGE : ORIGIN_CAPS);
}

/* 会话结束（充电器硬复位、退出、超时）：与 PD 前端 Hard Reset 相同处理，再尝试重连 */
static bool on_dead(void)
{
    has_contract = false;
    num_caps = 0;
    pending_valid = false;
    if (req_status == FE_REQ_BUSY)
        req_status = FE_REQ_FAIL;
    bridge_on_front_reset();
    ufcs_close();
    delay_ms(30);
    if (!ufcs_connect(true))
    {
        st = FU_OFF;
        return false;
    }
    st = FU_WAIT_CAPS;
    return true;
}

bool feu_process(void)
{
    if (afcn)
    {
        afc_process();
        if (st == FU_WAIT_CAPS)
        {
            on_caps();
            if (st == FU_WAIT_CAPS)
                st = FU_READY;
        }
        else if (st == FU_REQ)
        {
            if (pending_valid)
            {
                int8_t k = -1;
                for (uint8_t i = 0; i < afcn; i++)
                    if (afc_mv(afcl[i]) == pending.mv)
                        k = (int8_t)i;
                if (k < 0 || pending.type != FPDO)
                    request_finished(false);
                else
                {
                    afc_set(afcl[k]);
                    pending_valid = false;
                }
            }
            else if (!afc_busy())
                request_finished(afc_ok());
        }
        return true;
    }
    if (qcf)
    {
        qc_process();
        if (st == FU_WAIT_CAPS)
        {
            on_caps();
            if (st == FU_WAIT_CAPS)
                st = FU_READY;
        }
        else if (st == FU_REQ)
        {
            if (pending_valid)
            {
                if (pending.pos < 1 || pending.pos > num_caps)
                    request_finished(false);
                else
                {
                    qc_set(pending.mv, pending.type == PPS_PDO);
                    pending_valid = false;
                }
            }
            else if (!qc_busy())
                request_finished(true);
        }
        /* 跨档：等协议桥空闲（设备合约已更新）后再重新广播，保持当前合约 */
        else if (st == FU_READY && qc_one_pps() && bridge_flash_safe())
        {
            /* 按设备请求的电压（不含压降补偿）定档；设备未接时回到 3A 档 */
            uint32_t rdo;
            uint16_t mv = 5000;
            bridge_back_contract(&rdo, &mv);
            uint8_t b = qc_band_of(mv);
            if (b != qband)
            {
                qband = b;
                on_caps();
            }
        }
        return true;
    }
    ufcs_process();
    if (ufcs_dead())
        return on_dead();

    if (ufcs_take_caps())
        on_caps();

    if (st == FU_REQ)
    {
        if (pending_valid)
        {
            uint8_t mode;
            uint16_t mv, ma;
            if (!map_target(&pending, &mode, &mv, &ma))
                request_finished(false);
            else if (ufcs_request(mode, mv, ma))
                pending_valid = false;          /* 已发出，之后看 ufcs_req_status */
        }
        else
        {
            uint8_t s = ufcs_req_status();
            if (s == 1)
                request_finished(true);
            else if (s == 2)
                request_finished(false);
        }
    }
    return true;
}

bool feu_caps_available(void)
{
    return num_caps > 0;
}

uint32_t feu_contract_rdo(void)
{
    return has_contract ? contract_rdo : 0;
}

bool feu_flash_safe(void)
{
    return st == FU_READY && (afcn ? !afc_busy() : qcf ? !qc_busy() : ufcs_flash_safe());
}

uint8_t feu_state_code(void)
{
    return (afcn ? 24 : qcf ? 20 : 16) + (uint8_t)st;
}

bool feu_is_ready(void)
{
    return st == FU_READY && has_contract;
}

const pdo_t *feu_caps(uint8_t *num)
{
    *num = num_caps;
    return caps;
}

const fe_target_t *feu_contract(void)
{
    return has_contract ? &contract : NULL;
}

bool feu_request(const fe_target_t *t)
{
    if (!feu_is_ready())
        return false;
    begin_request(t, ORIGIN_BRIDGE);
    return true;
}

fe_req_status_t feu_request_status(void)
{
    return req_status;
}
