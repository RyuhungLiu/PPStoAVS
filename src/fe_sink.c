#include "fe_sink.h"
#include "analog.h"
#include "board.h"
#include "bridge.h"
#include "cfg.h"
#include "evlog.h"
#include "fe_ufcs.h"
#include "pdinfo.h"
#include "pd_phy.h"
#include "timebase.h"

#define T_SINK_WAIT_CAP_MS      1000    /* 规范 tTypeCSinkWaitCap 310~620ms，上电有调试窗口，放宽 */
#define T_LEGACY_MS             3000    /* Hard Reset 后仍无能力报文 → 按非 PD 充电器处理 */
#define T_SENDER_RESPONSE_MS    30
#define T_PS_TRANSITION_MS      550
#define T_AVS_TRANSITION_MS     750     /* AVS 大步进 tAvsSrcTransLarge ≤ 700ms */
#define T_SINK_TX_WAIT_MS       100     /* SinkTxNG 持续过久时仍然发送，避免死锁 */
#define T_KEEPALIVE_NG_MS       1000    /* 保活到期后 SinkTxNG 持续过久也照发（tPPSTimeout ≥ 12s） */
#define SINK_TX_OK_MV           1230    /* vRd-1.5 ≤ 1.16V，vRd-3.0 ≥ 1.31V */
#define T_SINK_REQUEST_MS       100     /* tSinkRequest：收到 Wait 后等待再重发 */
#define N_WAIT_RETRY            3
#define T_KEEPALIVE_RETRY_MS    1000    /* 保活请求失败后的重试间隔，避免连续重发 */
#define T_ENTER_EPR_MS          550     /* tEnterEPR 450~550ms（含充电器查询线材） */
#define T_EPR_KEEPALIVE_MS      250     /* tSinkEPRKeepAlive 250~500ms：这么久没发报文就发 EPR_KeepAlive */
#define T_EPR_KEEPALIVE_NG_MS   400     /* SinkTxNG 持续时也要在 tSinkEPRKeepAlive 上限前发出 */
#define N_EPR_ATTEMPTS          2       /* 每次上电最多尝试进入 EPR 的次数（避免失败循环） */
#define EPR_SINK_PDP            100     /* EPR_Mode (Enter) 的 EPR Sink Operational PDP（W） */

/* 参考手册 §15.2.15：CC 比较器 */
#define PORT_CE                 (1u << 7)
#define PORT_CVS_Mask           (3u << 5)
#define PORT_CVS_066            (2u << 5)
#define PORT_CVS_123            (3u << 5)

/* 参考手册 §6.3 AFIO_PCFR1[26:24] SWCFG=100：关闭 SDI，PA3 作为普通 IO / CC4R */
#define AFIO_SWCFG_Mask         (7u << 24)
#define AFIO_SWCFG_DISABLE      (4u << 24)

typedef enum
{
    FE_ST_WAIT_CAPS,
    FE_ST_WAIT_ACCEPT,
    FE_ST_WAIT_PS_RDY,
    FE_ST_READY,
    FE_ST_LEGACY,
    FE_ST_WAIT_RETRY,   /* 充电器回复 Wait，tSinkRequest 后重发同一请求 */
    FE_ST_SOFT_RESET,   /* 已发出 Soft Reset，等待 Accept */
    FE_ST_EPR_ENTER,    /* 已发出 EPR_Mode (Enter)，等待 Enter Acknowledged */
    FE_ST_EPR_WAIT_OK,  /* 等待 Enter Succeeded（充电器此时查询线材） */
} fe_state_t;

typedef enum
{
    ORIGIN_CAPS,        /* 响应 Source_Capabilities */
    ORIGIN_BRIDGE,      /* 协议桥请求 */
    ORIGIN_KEEPALIVE,   /* PPS 保活 */
} req_origin_t;

static pd_phy_t *const phy = &pd_phy_fe;

static fe_state_t state;
static uint32_t state_ts;
static uint8_t query;               /* 信息透传：进行中的查询 PI_Q_*，0 = 无 */
static uint32_t alert_out;          /* 待转发给充电器的设备 Alert */
static uint32_t keepalive_ts;
static bool hard_reset_sent;
static uint8_t last_rx_id;
static uint8_t cable_last_rx_id;    /* 虚拟 E-Marker：SOP' 报文的 MessageID */

static pdo_t caps[PD_MAX_EPR_OBJS];
static uint8_t num_caps;

/* Lab（CFG_FE_EMARKER）：EPR 模式 */
static bool epr_mode;
static uint8_t epr_attempts;
static uint8_t epr_buf[PD_MAX_EPR_OBJS * 4];    /* EPR_Source_Capabilities 分块拼接 */
static uint8_t epr_len, epr_total;
static bool has_contract;
static uint32_t contract_rdo;
static uint32_t pending_rdo;
static fe_target_t contract;
static fe_target_t pending;
static req_origin_t pending_origin;
static uint8_t wait_count;          /* 当前请求收到 Wait 的次数 */

static bool bridge_req_waiting;     /* 协议桥请求等待 SinkTxOK */
static uint32_t bridge_req_ts;
static uint8_t txwait;              /* 协议桥请求等 SinkTxOK 的时间（记录用） */

/* 线材的协议层随 Hard Reset、连接复位 */
static void cable_reset(void)
{
    cable_last_rx_id = 0xFF;
    phy->tx_msg_id_sop1 = 0;
}
static fe_req_status_t req_status = FE_REQ_IDLE;

#ifdef FE_DIAG
/*
 * 诊断：等待能力报文期间
 *  - 每 400ms 在 CC_SEL=1/0 之间切换接收通道
 *  - 每 10ms 用 0.66V 比较器快速采样两个 CC 口，统计电平翻转（BMC 是否到达引脚）
 *  - 不发 Hard Reset，3s 后直接进入 LEGACY 以便把计数编码进后端 PDO
 */
uint16_t fe_dbg_v[6];
static uint32_t dbg_smp_ts;

/*
 * 诊断：等待能力报文期间，每 10ms 快速轮询 USBPD1 自身的接收状态
 *  v[0] IF_RX_BIT 置位次数   v[1] IF_RX_BYTE 置位次数   v[2] 出现过的 RX_STATE 位图
 *  v[3] PA3 0.66V 比较器翻转  v[4] CONTROL 寄存器        v[5] (CONFIG >> 6) & 0x3FF
 *  不发 Hard Reset，3s 后进入 LEGACY 把计数编码进后端 PDO
 */
static void diag_wait_caps(void)
{
    uint32_t now = millis();
    if (now - dbg_smp_ts < 10)
        return;
    dbg_smp_ts = now;

    USBPD_TypeDef *r = phy->regs;
    for (uint16_t i = 0; i < 400; i++)
    {
        uint8_t st = r->STATUS;
        if (st & IF_RX_BIT)
        {
            fe_dbg_v[0]++;
            r->STATUS = IF_RX_BIT;
        }
        if (st & IF_RX_BYTE)
        {
            fe_dbg_v[1]++;
            r->STATUS = IF_RX_BYTE;
        }
        fe_dbg_v[2] |= 1u << ((r->CONTROL >> 2) & 7);
    }

    volatile uint8_t *reg = pd_phy_port_reg(phy, 1);
    uint8_t saved = *reg;
    *reg = (saved & ~PORT_CVS_Mask) | PORT_CE | PORT_CVS_066;
    delay_us(2);
    uint8_t lastc = *reg & CC_CMPO;
    for (uint16_t i = 0; i < 400; i++)
    {
        uint8_t v = *reg & CC_CMPO;
        if (v != lastc)
        {
            fe_dbg_v[3]++;
            lastc = v;
        }
    }
    *reg = saved;

    fe_dbg_v[4] = r->CONTROL;
    fe_dbg_v[5] = (r->CONFIG >> 6) & 0x3FF;
}
#endif

static void set_state(fe_state_t s)
{
    if (s != FE_ST_READY)
        query = 0;      /* 被打断的查询留在队列里，回到 READY 后再问 */
    state = s;
    state_ts = millis();
}

static volatile uint8_t *cc_port(void)
{
    return pd_phy_port_reg(phy, 1);   /* USBPD1 的 PORT_CC2 = CC4 */
}

static bool cc_above(uint8_t cvs)
{
    volatile uint8_t *reg = cc_port();
    *reg = (*reg & ~PORT_CVS_Mask) | PORT_CE | cvs;
    delay_us(5);
    bool above = (*reg & CC_CMPO) != 0;
    *reg = (*reg & ~PORT_CVS_Mask) | PORT_CE | PORT_CVS_066;   /* 恢复接收用的比较器设置 */
    return above;
}

/*
 * PD3.0 冲突避免：源端 Rp=3A 表示 SinkTxOK，1.5A 表示 SinkTxNG。
 * 用 ADC（PA3/ADC_IN16）量 CC 电压，不改接收用的比较器门限，不会破坏正在接收的报文；
 * 正好采在 BMC 报文上时读数低于门限，按 NG 处理（此时本来也不该发送）
 */
static bool sink_tx_ok(void)
{
    if (phy->revision < PD_REV_30)
        return true;
    return analog_fe_cc_mv() >= SINK_TX_OK_MV;
}

static void send_ctrl(uint8_t type)
{
    pd_phy_send(phy, type, 0, NULL);
}

/* PD3.0 回 Not_Supported，PD2.0 没有该消息，回 Reject */
static void send_not_supported(void)
{
    send_ctrl(phy->revision >= PD_REV_30 ? MSG_TYPE_Not_Supported : MSG_TYPE_Reject);
}

static bool epr_wanted(void)
{
    return (cfg()->flags & CFG_FE_EMARKER) != 0;
}

/* EPR 模式下用 EPR_Request（RDO + 所请求 PDO 的副本），否则 Request */
static void send_rdo(uint32_t rdo, uint8_t pos)
{
    if (epr_mode && pos >= 1 && pos <= num_caps)
    {
        uint32_t objs[2] = {rdo, caps[pos - 1].raw};
        pd_phy_send(phy, MSG_TYPE_EPR_Request, 2, objs);
    }
    else
    {
        pd_phy_send(phy, MSG_TYPE_Request, 1, &rdo);
    }
}

static void send_request(const fe_target_t *t, req_origin_t origin)
{
    uint32_t rdo;
    if (t->type == PPS_PDO)
        rdo = pd_build_pps_rdo(t->pos, t->mv, t->ma);
    else if (t->type == SPR_AVS_PDO || t->type == EPR_AVS_PDO)
        rdo = pd_build_avs_rdo(t->pos, t->mv, t->ma);   /* AVS 不需要保活 */
    else
        rdo = pd_build_fixed_rdo(t->pos, t->ma, t->ma);
    if (epr_wanted())
        rdo |= RDO_EPR_CAPABLE;

    pending = *t;
    pending_rdo = rdo;
    pending_origin = origin;
    wait_count = 0;
    send_rdo(rdo, t->pos);
    set_state(FE_ST_WAIT_ACCEPT);
}

static void resend_pending(void)
{
    send_rdo(pending_rdo, pending.pos);
    set_state(FE_ST_WAIT_ACCEPT);
}

static void log_note(uint8_t code, uint8_t data)
{
    ev_note_t e = {0, code, data};
    evlog_add(EV_NOTE, &e, sizeof(e));
}

static void epr_reset(void)
{
    epr_mode = false;
    epr_len = epr_total = 0;
}

static void request_finished(bool ok)
{
    if (ok)
    {
        contract = pending;
        contract_rdo = pending_rdo;
        has_contract = true;
        pdinfo_chg_start(phy->revision >= PD_REV_30);
        hard_reset_sent = false;
        keepalive_ts = millis();
    }
    else if (pending_origin == ORIGIN_KEEPALIVE)
    {
        keepalive_ts = millis() - PPS_KEEPALIVE_MS + T_KEEPALIVE_RETRY_MS;
    }
    if (pending_origin == ORIGIN_CAPS)
    {
        ev_fe_contract_t e = {pending_rdo, pending.mv, ok};
        evlog_add(EV_FE_CONTRACT, &e, sizeof(e));
    }
    if (pending_origin == ORIGIN_BRIDGE)
    {
        req_status = ok ? FE_REQ_OK : FE_REQ_FAIL;
    }
    set_state(FE_ST_READY);
}

static void log_reset(uint8_t kind)
{
    ev_reset_t e = {0, kind, (uint8_t)state, phy->dbg_rx_reset};
    evlog_add(EV_RESET, &e, sizeof(e));
}

/*
 * 等不到 Accept：先发 Soft Reset（不改变电压），充电器重新广播后协议桥重发请求；
 * Soft Reset 也没有 Accept 才 Hard Reset
 */
static void send_soft_reset(void)
{
    log_reset(RST_SOFT_SENT);
    pd_phy_reset_protocol(phy);
    last_rx_id = 0xFF;
    if (req_status == FE_REQ_BUSY)
        req_status = FE_REQ_ABORTED;
    bridge_req_waiting = false;
    send_ctrl(MSG_TYPE_Soft_Reset);
    set_state(FE_ST_SOFT_RESET);
}

static void do_hard_reset(void)
{
    log_reset(RST_HARD_SENT);
    pd_phy_send_hard_reset(phy);
    pd_phy_reset_protocol(phy);
    cable_reset();
    epr_reset();
    pdinfo_chg_reset();
    last_rx_id = 0xFF;
    has_contract = false;
    num_caps = 0;
    if (pending_origin == ORIGIN_BRIDGE && req_status == FE_REQ_BUSY)
        req_status = FE_REQ_FAIL;
    bridge_on_front_reset();
    set_state(FE_ST_WAIT_CAPS);
}

/* Source_Capabilities 或 EPR_Source_Capabilities（1~7 SPR，8~ EPR，空位为 0） */
static void handle_caps(const uint8_t *data, uint8_t n, uint8_t revision)
{
    uint32_t raw[PD_MAX_EPR_OBJS];
    num_caps = n;
    for (uint8_t i = 0; i < num_caps; i++)
    {
        raw[i] = pd_get_u32(&data[i * 4]);
        caps[i] = pd_parse_pdo(raw[i]);
    }
    evlog_add(EV_FE_CAPS, raw, num_caps * 4);
    phy->revision = (revision < PD_REV_30) ? revision : PD_REV_30;

    /* 如有协议桥请求正在进行，被新的能力报文打断（协议桥可重发） */
    if (req_status == FE_REQ_BUSY)
        req_status = FE_REQ_ABORTED;
    bridge_req_waiting = false;

    bool for_bridge;
    fe_target_t t = bridge_on_front_caps(&for_bridge);
    if (for_bridge)
    {
        /* 转换进行中（如充电器 Soft Reset 后重新广播）：直接用设备的新目标回应，回应能力报文不受 SinkTx 限制 */
        req_status = FE_REQ_BUSY;
        send_request(&t, ORIGIN_BRIDGE);
    }
    else
    {
        send_request(&t, ORIGIN_CAPS);
    }
}

static void handle_source_caps(const pd_rx_msg_t *m, const pd_header_t *h)
{
    if (epr_mode)
    {
        epr_mode = false;       /* 充电器改发 SPR 能力：已退出 EPR */
        log_note(NOTE_FE_EPR_EXIT, 0);
    }
    handle_caps(&m->data[2], h->num_objs, h->revision);
}

/* ---- 信息透传：查询充电器、代答充电器对设备的查询 ---- */

static bool send_query(uint8_t q)
{
    static const uint8_t port_ref[2] = {0, 0};     /* Manufacturer Info Target = 端口 */
    uint32_t vdm = VDM_DISC_IDENT_REQ;
    switch (q)
    {
    case PI_Q_IDENT:
        return pd_phy_send(phy, MSG_TYPE_Vendor_Defined, 1, &vdm);
    case PI_Q_EXTCAPS:
        return pd_phy_send(phy, MSG_TYPE_Get_Source_Cap_Extended, 0, NULL);
    case PI_Q_SIDO:
        return pd_phy_send(phy, MSG_TYPE_Get_Source_Info, 0, NULL);
    case PI_Q_STATUS:
        return pd_phy_send(phy, MSG_TYPE_Get_Status, 0, NULL);
    default:
        return pd_phy_send_ext(phy, MSG_TYPE_Get_Manufacturer_Info, port_ref, sizeof(port_ref));
    }
}

static void query_end(void)
{
    pdinfo_chg_query_done(query);
    query = 0;
    set_state(FE_ST_READY);
}

/* 收到对查询 q 的应答（或拒绝）时结束查询 */
static bool query_is(uint8_t q)
{
    return state == FE_ST_READY && query == q;
}

static void send_sink_caps_ext(void);

static void answer_sink_caps_ext(void)
{
    uint8_t d[SKEDB_LEN];
    if (pdinfo_fe_skedb(d))
    {
        pd_phy_send_ext(phy, MSG_TYPE_Sink_Capabilities_Extended, d, sizeof(d));
        if (cfg()->flags & CFG_FE_AVS_2ND)
        {
            ev_avs_2nd_t e = {0, AVS2_FE_SKEDB_SENT, d[SKEDB_SINK_MODES]};
            evlog_add(EV_AVS_2ND, &e, sizeof(e));
        }
    }
    else if (cfg()->flags & CFG_FE_AVS_2ND)
    {
        send_sink_caps_ext();
    }
    else
    {
        send_not_supported();
    }
}

static void handle_vdm(const pd_rx_msg_t *m, uint8_t n)
{
    uint32_t vdm = pd_get_u32(&m->data[2]);
    if (!VDM_STRUCTURED(vdm))
        return;
    if (VDM_CMD_TYPE(vdm) != 0)
    {
        /* 充电器对 Discover Identity 的应答 */
        if (query_is(PI_Q_IDENT) && VDM_IS_DISC_IDENT(vdm) && VDM_CMD_TYPE(vdm) != VDM_BUSY)
        {
            if (VDM_CMD_TYPE(vdm) == VDM_ACK && n > 1)
            {
                uint32_t vdo[PI_MAX_VDOS];
                uint8_t k = n - 1 > PI_MAX_VDOS ? PI_MAX_VDOS : n - 1;
                for (uint8_t i = 0; i < k; i++)
                    vdo[i] = pd_get_u32(&m->data[6 + 4 * i]);
                pdinfo_chg_ident(vdo, k);
            }
            query_end();
        }
        return;
    }
    if (VDM_IS_DISC_IDENT(vdm) && (pdinfo_id_on() || pdinfo_fe_custom()))
    {
        /* 身份透传：回设备的身份（自订身份：回自订的 Sink 身份）；设备的还没读到就回 BUSY 让充电器稍后再问 */
        static uint8_t logged_reply;
        pi_ident_t id;
        uint32_t o[1 + PI_MAX_VDOS];
        uint8_t reply;
        if (pdinfo_fe_ident(&id))
        {
            reply = VDM_ACK;
            o[0] = VDM_REPLY(vdm, VDM_ACK);
            memcpy(&o[1], id.vdo, id.n * 4);
            pd_phy_send(phy, MSG_TYPE_Vendor_Defined, 1 + id.n, o);
        }
        else
        {
            reply = pdinfo_fe_ident_pending() ? VDM_BUSY : VDM_NAK;
            o[0] = VDM_REPLY(vdm, reply);
            pd_phy_send(phy, MSG_TYPE_Vendor_Defined, 1, o);
        }
        if (reply != logged_reply)
        {
            logged_reply = reply;
            log_note(NOTE_FE_IDENT, reply);
        }
        return;
    }
    send_not_supported();
}

/* 扩展报文：EPR_Source_Capabilities 分块拼接、Extended_Control，其余回 Not_Supported */
static void handle_ext(const pd_rx_msg_t *m, const pd_header_t *h)
{
    uint16_t ext = m->data[2] | (m->data[3] << 8);
    const uint8_t *d = &m->data[4];
    uint8_t avail = m->len > 4 ? m->len - 4 : 0;

    if (h->msg_type == MSG_TYPE_EPR_Source_Capabilities)
    {
        if (ext & EXT_REQUEST_CHUNK)
            return;
        uint8_t chunk = EXT_CHUNK_NUM(ext);
        if (chunk == 0)
        {
            uint16_t size = EXT_DATA_SIZE(ext);
            epr_total = size > sizeof(epr_buf) ? sizeof(epr_buf) : size;
            epr_len = 0;
        }
        else if (epr_total == 0 || epr_len != chunk * EXT_CHUNK_BYTES)
        {
            return;     /* 分块顺序不对，等充电器重发 */
        }
        uint8_t n = epr_total - epr_len;
        if (n > EXT_CHUNK_BYTES)
            n = EXT_CHUNK_BYTES;
        if (n > avail)
            n = avail;
        memcpy(&epr_buf[epr_len], d, n);
        epr_len += n;
        if (epr_len < epr_total)
        {
            pd_phy_send_chunk_request(phy, MSG_TYPE_EPR_Source_Capabilities, chunk + 1);
            return;
        }
        epr_mode = true;
        uint8_t objs = epr_total / 4;
        epr_total = epr_len = 0;
        handle_caps(epr_buf, objs, h->revision);
        return;
    }
    if (h->msg_type == MSG_TYPE_Extended_Control && avail >= 1 && d[0] == ECDB_EPR_KEEPALIVE_ACK)
        return;
    if (ext & EXT_REQUEST_CHUNK)
        return;

    uint8_t size = EXT_DATA_SIZE(ext) < avail ? EXT_DATA_SIZE(ext) : avail;
    switch (h->msg_type)
    {
    case MSG_TYPE_Source_Capabilities_Extended:
        if (query_is(PI_Q_EXTCAPS))
        {
            pdinfo_chg_scedb(d, size);
            query_end();
        }
        return;
    case MSG_TYPE_Status:
        if (query_is(PI_Q_STATUS))
        {
            pdinfo_chg_status(d, size);
            query_end();
        }
        return;
    case MSG_TYPE_Manufacturer_Info:
        if (query_is(PI_Q_MIDB))
        {
            pdinfo_chg_midb(d, size);
            query_end();
        }
        return;
    case MSG_TYPE_Get_Battery_Cap:
        if (pdinfo_on())
        {
            uint8_t bcdb[9];
            pdinfo_fe_bcap(size ? d[0] : 0, bcdb);
            pd_phy_send_ext(phy, MSG_TYPE_Battery_Capabilities, bcdb, sizeof(bcdb));
            return;
        }
        break;
    case MSG_TYPE_Get_Battery_Status:
        if (pdinfo_on())
        {
            uint32_t bsdo = pdinfo_fe_bsdo(size ? d[0] : 0);
            pd_phy_send(phy, MSG_TYPE_Battery_Status, 1, &bsdo);
            return;
        }
        break;
    default:
        break;
    }
    send_not_supported();
}

static void send_epr_keepalive(void)
{
    uint8_t d[2] = {ECDB_EPR_KEEPALIVE, 0};
    pd_phy_send_ext(phy, MSG_TYPE_Extended_Control, d, sizeof(d));
}

static bool caps_epr_capable(void)
{
    return num_caps > 0 && caps[0].type == FPDO && (caps[0].raw & PDO_EPR_CAPABLE);
}

static void send_sink_caps(void)
{
    uint32_t objs[2];
    uint16_t ma = bridge_back_max_ma();
    objs[0] = pd_build_fixed_pdo(5000, ma, 0);
    objs[1] = pd_build_pps_apdo(3300, 21000, ma);
    pd_phy_send(phy, MSG_TYPE_Sink_Capabilities, 2, objs);
}

/*
 * Lab（CFG_FE_AVS_2ND）：充电器查询 Sink_Capabilities_Extended 时声明支持 AVS（Sink Modes bit5）。
 * 支持二次握手的充电器会重新广播，用 SPR AVS 取代 PPS（实测抓包：先 PPS → 查询 → 改广播 AVS）。
 */
static void send_sink_caps_ext(void)
{
    const cfg_t *c = cfg();
    uint8_t d[SKEDB_LEN] = {0};
    uint8_t pdp = (uint32_t)c->max_mv * bridge_back_max_ma() / 1000000;
    d[0] = 0x09;                    /* VID 0x1209 */
    d[1] = 0x12;
    d[2] = 0x01;                    /* PID 0x0001 */
    d[8] = FW_VERSION & 0xFF;       /* FW Version */
    d[9] = 1;                       /* HW Version */
    d[10] = 1;                      /* SKEDB Version */
    d[SKEDB_SINK_MODES] = SINK_MODE_AVS;    /* 与实测支持 AVS 的受电端一致，只声明 AVS */
    d[18] = 15;                     /* SPR Sink Minimum PDP：5V 3A */
    d[19] = pdp;                    /* SPR Sink Operational PDP */
    d[20] = pdp;                    /* SPR Sink Maximum PDP */
    pd_phy_send_ext(phy, MSG_TYPE_Sink_Capabilities_Extended, d, sizeof(d));
    ev_avs_2nd_t e = {0, AVS2_FE_SKEDB_SENT, d[SKEDB_SINK_MODES]};
    evlog_add(EV_AVS_2ND, &e, sizeof(e));
}

/*
 * Lab（CFG_FE_EMARKER）：扮演无源 5A 线材（E-Marker），应答充电器 SOP' 的 Discover Identity。
 * 格式参照实测 5A 线材：ID Header 产品类型 = 无源线材；Cable VDO：USB-C 对 USB-C、50V、5A、EPR、USB 2.0。
 * 充电器在 VCONN 脚检测到 Ra 才会供 VCONN、查询线材：公头 B5 经 1kΩ 接 PA2，PA2 拉低即为 Ra
 */
void fe_ra_apply(void)
{
    static int8_t applied = -1;
    int8_t on = (cfg()->flags & CFG_FE_EMARKER) != 0;
    if (on == applied)
        return;
    applied = on;
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef gpio = {0};
    gpio.GPIO_Pin = FE_RA_PIN;
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    if (on)
    {
        GPIOA->BCR = FE_RA_PIN;
        gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    }
    else
    {
        gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;     /* 高阻：没有 Ra，充电器不供 VCONN */
    }
    GPIO_Init(GPIOA, &gpio);
}
#define EMARKER_ID_HEADER       ((3u << 27) | 0x1209u)      /* 无源线材，VID 0x1209 */
#define EMARKER_PRODUCT_VDO     (0x0001u << 16)             /* PID 0x0001 */
/* C-C、EPR Capable、延迟 <10ns、50V、5A、USB 2.0（EPR 要求线材 50V/5A/EPR；本机只请求 ≤ 20V） */
#define EMARKER_CABLE_VDO       ((2u << 18) | (1u << 17) | (1u << 13) | (3u << 9) | (2u << 5))

static void handle_cable_msg(const pd_rx_msg_t *m)
{
    pd_header_t h = pd_parse_header(m->data[0] | (m->data[1] << 8));
    uint8_t rev = h.revision < PD_REV_30 ? h.revision : PD_REV_30;

    if (!h.extended && h.num_objs == 0 && h.msg_type == MSG_TYPE_Soft_Reset)
    {
        cable_reset();
        pd_phy_send_sop1(phy, MSG_TYPE_Accept, 0, NULL, rev);
        return;
    }
    if (h.msg_id == cable_last_rx_id)
        return;
    cable_last_rx_id = h.msg_id;
    if (h.extended || h.msg_type != MSG_TYPE_Vendor_Defined || h.num_objs == 0)
        return;

    uint32_t vdm = pd_get_u32(&m->data[2]);
    if (!((vdm >> 15) & 1) || ((vdm >> 6) & 3) != 0)
        return;     /* 只应答结构化 VDM 的 REQ */
    if ((vdm >> 16) == 0xFF00 && (vdm & 0x1F) == 1)
    {
        uint32_t o[5] = {(vdm & ~0xC0u) | 0x40u, EMARKER_ID_HEADER, 0, EMARKER_PRODUCT_VDO, EMARKER_CABLE_VDO};
        if (pd_phy_send_sop1(phy, MSG_TYPE_Vendor_Defined, 5, o, rev))
            log_note(NOTE_FE_EMARKER, 0);
    }
    else
    {
        uint32_t nak = (vdm & ~0xC0u) | 0x80u;
        pd_phy_send_sop1(phy, MSG_TYPE_Vendor_Defined, 1, &nak, rev);
    }
}

static void handle_msg(const pd_rx_msg_t *m)
{
    if (m->sop)
    {
        handle_cable_msg(m);
        return;
    }
    pd_header_t h = pd_parse_header(m->data[0] | (m->data[1] << 8));

    if (!h.extended && h.num_objs == 0 && h.msg_type == MSG_TYPE_Soft_Reset)
    {
        log_reset(RST_SOFT_RCVD);
        pd_phy_reset_protocol(phy);
        last_rx_id = 0xFF;
        send_ctrl(MSG_TYPE_Accept);
        if (req_status == FE_REQ_BUSY)
            req_status = FE_REQ_ABORTED;
        bridge_req_waiting = false;
        set_state(FE_ST_WAIT_CAPS);
        return;
    }
    if (h.msg_id == last_rx_id)
        return;     /* 重发的报文 */
    last_rx_id = h.msg_id;

    if (h.extended)
    {
        handle_ext(m, &h);
        return;
    }

    if (h.num_objs > 0)
    {
        switch (h.msg_type)
        {
        case MSG_TYPE_Source_Capabilities:
            handle_source_caps(m, &h);
            break;
        case MSG_TYPE_Vendor_Defined:
            handle_vdm(m, h.num_objs);
            break;
        case MSG_TYPE_Source_Info:
            if (query_is(PI_Q_SIDO))
            {
                pdinfo_chg_sido(pd_get_u32(&m->data[2]));
                query_end();
            }
            break;
        case MSG_TYPE_EPR_Mode:
        {
            uint32_t d = pd_get_u32(&m->data[2]);
            uint8_t action = d >> 24, data = (d >> 16) & 0xFF;
            if (action == EPR_MODE_ENTER_ACK && state == FE_ST_EPR_ENTER)
            {
                set_state(FE_ST_EPR_WAIT_OK);
            }
            else if (action == EPR_MODE_ENTER_OK && (state == FE_ST_EPR_ENTER || state == FE_ST_EPR_WAIT_OK))
            {
                epr_mode = true;
                log_note(NOTE_FE_EPR_ENTERED, 0);
                set_state(FE_ST_WAIT_CAPS);     /* 充电器接着发 EPR_Source_Capabilities */
            }
            else if (action == EPR_MODE_ENTER_FAILED)
            {
                log_note(NOTE_FE_EPR_FAILED, data);
                set_state(FE_ST_READY);
            }
            else if (action == EPR_MODE_EXIT)
            {
                epr_mode = false;
                log_note(NOTE_FE_EPR_EXIT, 0);
                set_state(FE_ST_WAIT_CAPS);     /* 充电器接着发 SPR 能力 */
            }
            break;
        }
        case MSG_TYPE_Alert:
        case MSG_TYPE_BIST:
            break;
        default:
            send_not_supported();
            break;
        }
        return;
    }

    switch (h.msg_type)
    {
    case MSG_TYPE_Accept:
        if (state == FE_ST_WAIT_ACCEPT)
            set_state(FE_ST_WAIT_PS_RDY);
        else if (state == FE_ST_SOFT_RESET)
            set_state(FE_ST_WAIT_CAPS);
        break;
    case MSG_TYPE_Reject:
    case MSG_TYPE_Wait:
        if (query && h.msg_type == MSG_TYPE_Reject)
            query_end();
        else if (state == FE_ST_WAIT_ACCEPT)
        {
            if (h.msg_type == MSG_TYPE_Wait && wait_count < N_WAIT_RETRY)
            {
                wait_count++;
                set_state(FE_ST_WAIT_RETRY);    /* 充电器忙（如请求过快），稍后重发 */
            }
            else if (has_contract)
                request_finished(false);
            else
                do_hard_reset();
        }
        break;
    case MSG_TYPE_PS_RDY:
        if (state == FE_ST_WAIT_PS_RDY)
        {
            request_finished(true);
        }
        else if (state == FE_ST_WAIT_ACCEPT)
        {
            /* 漏收了 Accept：PS_RDY 说明充电器已接受并完成转换 */
            log_note(NOTE_FE_ACCEPT_MISSED, 0);
            request_finished(true);
        }
        break;
    case MSG_TYPE_Get_Sink_Cap:
        send_sink_caps();
        break;
    case MSG_TYPE_Get_Sink_Cap_Extended:
        answer_sink_caps_ext();
        break;
    case MSG_TYPE_Not_Supported:
        if (query)
            query_end();
        break;
    case MSG_TYPE_GoodCRC:
    case MSG_TYPE_GotoMin:
    case MSG_TYPE_Ping:
        break;
    default:
        send_not_supported();
        break;
    }
}

void fe_init(void)
{
    /* 关闭 SDI，PA3 交给 USBPD1 */
    AFIO->PCFR1 = (AFIO->PCFR1 & ~AFIO_SWCFG_Mask) | AFIO_SWCFG_DISABLE;

    pd_phy_init(phy, USBPD1, USBPD1_IRQn, 0 /* Sink */, 0 /* UFP */);
    phy->regs->PORT_CC1 = 0;                                  /* CC3 不使用 */
    /* CC4R：Rd；BMC 接收经 CC 比较器，接收期间保持 CE=1、0.66V（WCH EVT USBPD_SNK/SRC 均如此） */
    phy->regs->PORT_CC2 = CC_PD | PORT_CE | PORT_CVS_066;
    pd_phy_set_cc(phy, 1);

    last_rx_id = 0xFF;
    cable_reset();
    pdinfo_chg_reset();
    has_contract = false;
    num_caps = 0;
    hard_reset_sent = false;
    set_state(FE_ST_WAIT_CAPS);
}

static void enter_legacy(void)
{
    set_state(FE_ST_LEGACY);
    uint16_t ma = fe_legacy_current_ma();
    evlog_add(EV_FE_LEGACY, &ma, sizeof(ma));
    bool for_bridge;
    bridge_on_front_caps(&for_bridge);
}

void fe_process(void)
{
    if (feu_active())
    {
        /* UFCS 前端：会话丢失且重连失败时改用 PD */
        if (!feu_process())
            fe_init();
        return;
    }
    phy->sop1_en = (cfg()->flags & CFG_FE_EMARKER) != 0;
    fe_ra_apply();
    if (phy->hard_reset_rcvd)
    {
        phy->hard_reset_rcvd = false;
        log_reset(RST_HARD_RCVD);
        pd_phy_reset_protocol(phy);
        cable_reset();
        epr_reset();
        pdinfo_chg_reset();
        last_rx_id = 0xFF;
        has_contract = false;
        num_caps = 0;
        if (req_status == FE_REQ_BUSY)
            req_status = FE_REQ_FAIL;
        bridge_on_front_reset();
        set_state(FE_ST_WAIT_CAPS);
    }

    pd_rx_msg_t m;
    while (pd_phy_rx_pop(phy, &m))
    {
        handle_msg(&m);
    }

    uint32_t elapsed = millis() - state_ts;
    switch (state)
    {
    case FE_ST_WAIT_CAPS:
#ifdef FE_DIAG
        diag_wait_caps();
        if (elapsed > T_LEGACY_MS)
        {
            enter_legacy();
        }
        break;
#endif
        if (!hard_reset_sent && elapsed > T_SINK_WAIT_CAP_MS)
        {
            hard_reset_sent = true;
            do_hard_reset();
        }
        else if (hard_reset_sent && elapsed > T_LEGACY_MS)
        {
            enter_legacy();
        }
        break;

    case FE_ST_WAIT_ACCEPT:
        if (elapsed > T_SENDER_RESPONSE_MS)
        {
            if (has_contract)
                send_soft_reset();
            else
                do_hard_reset();
        }
        break;

    case FE_ST_SOFT_RESET:
        if (elapsed > T_SENDER_RESPONSE_MS)
            do_hard_reset();
        break;

    case FE_ST_EPR_ENTER:
        if (elapsed > T_SENDER_RESPONSE_MS)
            send_soft_reset();
        break;

    case FE_ST_EPR_WAIT_OK:
        if (elapsed > T_ENTER_EPR_MS)
            send_soft_reset();
        break;

    case FE_ST_WAIT_PS_RDY:
    {
        bool avs = pending.type == SPR_AVS_PDO || pending.type == EPR_AVS_PDO;
        if (elapsed > (avs ? T_AVS_TRANSITION_MS : T_PS_TRANSITION_MS))
            do_hard_reset();
        break;
    }

    case FE_ST_WAIT_RETRY:
        if (elapsed >= T_SINK_REQUEST_MS && (sink_tx_ok() || elapsed >= T_SINK_REQUEST_MS + T_SINK_TX_WAIT_MS))
            resend_pending();
        break;

    case FE_ST_READY:
        if (!alert_out)
            alert_out = pdinfo_fe_take_alert();
        if (query)
        {
            if (elapsed > T_SENDER_RESPONSE_MS)
                query_end();    /* 充电器没回：放弃这一项 */
        }
        else if (bridge_req_waiting)
        {
            uint32_t waited = millis() - bridge_req_ts;
            bool ok = sink_tx_ok();
            if (ok || waited > T_SINK_TX_WAIT_MS)
            {
                bridge_req_waiting = false;
                txwait = ok ? (uint8_t)waited : FE_TXWAIT_TIMEOUT;
                send_request(&pending, ORIGIN_BRIDGE);
            }
        }
        else if (has_contract && contract.type == PPS_PDO && millis() - keepalive_ts > PPS_KEEPALIVE_MS)
        {
            if (sink_tx_ok() || millis() - keepalive_ts > PPS_KEEPALIVE_MS + T_KEEPALIVE_NG_MS)
                send_request(&contract, ORIGIN_KEEPALIVE);
        }
        else if (epr_mode && millis() - phy->last_tx_ms >= T_EPR_KEEPALIVE_MS)
        {
            if (sink_tx_ok() || millis() - phy->last_tx_ms >= T_EPR_KEEPALIVE_NG_MS)
                send_epr_keepalive();
        }
        else if (epr_wanted() && !epr_mode && epr_attempts < N_EPR_ATTEMPTS && has_contract && caps_epr_capable() &&
                 elapsed > T_SENDER_RESPONSE_MS)
        {
            if (sink_tx_ok())
            {
                /* 进入 EPR：[31:24] Action = Enter，[23:16] EPR Sink Operational PDP */
                uint32_t d = ((uint32_t)EPR_MODE_ENTER << 24) | ((uint32_t)EPR_SINK_PDP << 16);
                epr_attempts++;
                pd_phy_send(phy, MSG_TYPE_EPR_Mode, 1, &d);
                set_state(FE_ST_EPR_ENTER);
            }
        }
        else if (alert_out && elapsed > T_SENDER_RESPONSE_MS && sink_tx_ok())
        {
            /* 设备电池状态变化：转发 Alert，充电器随后读取电池状态 */
            if (phy->revision >= PD_REV_30)
                pd_phy_send(phy, MSG_TYPE_Alert, 1, &alert_out);
            alert_out = 0;
        }
        else if (has_contract && elapsed > T_SENDER_RESPONSE_MS && pdinfo_chg_next_query() && sink_tx_ok())
        {
            query = pdinfo_chg_next_query();
            if (send_query(query))
                set_state(FE_ST_READY);     /* 从现在开始计 tSenderResponse */
            else
                query_end();
        }
        break;

    case FE_ST_LEGACY:
        break;
    }
}

bool fe_caps_available(void)
{
    if (feu_active())
        return feu_caps_available();
    return num_caps > 0 || state == FE_ST_LEGACY;
}

uint32_t fe_contract_rdo(void)
{
    if (feu_active())
        return feu_contract_rdo();
    return has_contract ? contract_rdo : 0;
}

bool fe_flash_safe(void)
{
    if (feu_active())
        return feu_flash_safe();
    if (state == FE_ST_LEGACY)
        return true;
    if (state != FE_ST_READY || bridge_req_waiting)
        return false;
    /* PPS 保活、EPR KeepAlive 快到期时不做（Flash 操作期间无法收发） */
    if (epr_mode && millis() - phy->last_tx_ms > T_EPR_KEEPALIVE_MS - 100)
        return false;
    return !(has_contract && contract.type == PPS_PDO && millis() - keepalive_ts > PPS_KEEPALIVE_MS - 200);
}

uint8_t fe_state_code(void)
{
    if (feu_active())
        return feu_state_code();
    return (uint8_t)state;
}

bool fe_is_ready(void)
{
    if (feu_active())
        return feu_is_ready();
    return state == FE_ST_READY && has_contract && !bridge_req_waiting;
}

bool fe_is_legacy(void)
{
    if (feu_active())
        return false;
    return state == FE_ST_LEGACY;
}

uint16_t fe_legacy_current_ma(void)
{
    if (cc_above(PORT_CVS_123))
        return 3000;
    if (cc_above(PORT_CVS_066))
        return 1500;
    return 500;
}

const pdo_t *fe_caps(uint8_t *num)
{
    if (feu_active())
        return feu_caps(num);
    *num = num_caps;
    return caps;
}

bool fe_caps_unconstrained(void)
{
    if (feu_active())
        return false;
    return num_caps > 0 && ((caps[0].raw >> 27) & 1);
}

const fe_target_t *fe_contract(void)
{
    if (feu_active())
        return feu_contract();
    return has_contract ? &contract : NULL;
}

bool fe_request(const fe_target_t *t)
{
    if (feu_active())
        return feu_request(t);
    if (!fe_is_ready())
        return false;
    pending = *t;
    pending_origin = ORIGIN_BRIDGE;
    req_status = FE_REQ_BUSY;
    bridge_req_waiting = true;
    bridge_req_ts = millis();
    txwait = 0;
    return true;
}

fe_req_status_t fe_request_status(void)
{
    if (feu_active())
        return feu_request_status();
    return req_status;
}

uint8_t fe_request_waits(void)
{
    if (feu_active())
        return 0;
    return wait_count;
}

uint8_t fe_request_txwait(void)
{
    if (feu_active())
        return 0;
    return txwait;
}

bool fe_epr_mode(void)
{
    if (feu_active())
        return false;
    return epr_mode;
}
