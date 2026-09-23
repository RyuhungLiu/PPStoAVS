#include "fe_sink.h"
#include "board.h"
#include "bridge.h"
#include "pd_phy.h"
#include "timebase.h"

#define T_SINK_WAIT_CAP_MS      1000    /* 规范 tTypeCSinkWaitCap 310~620ms，上电有调试窗口，放宽 */
#define T_LEGACY_MS             3000    /* Hard Reset 后仍无能力报文 → 按非 PD 充电器处理 */
#define T_SENDER_RESPONSE_MS    30
#define T_PS_TRANSITION_MS      550
#define T_SINK_TX_WAIT_MS       100     /* SinkTxNG 持续过久时仍然发送，避免死锁 */

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
static uint32_t keepalive_ts;
static bool hard_reset_sent;
static uint8_t last_rx_id;

static pdo_t caps[PD_MAX_DATA_OBJS];
static uint8_t num_caps;
static bool has_contract;
static fe_target_t contract;
static fe_target_t pending;
static req_origin_t pending_origin;

static bool bridge_req_waiting;     /* 协议桥请求等待 SinkTxOK */
static uint32_t bridge_req_ts;
static fe_req_status_t req_status = FE_REQ_IDLE;

static void set_state(fe_state_t s)
{
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
    *reg &= ~PORT_CE;
    return above;
}

/* PD3.0 冲突避免：源端 Rp=3A 表示 SinkTxOK，1.5A 表示 SinkTxNG */
static bool sink_tx_ok(void)
{
    if (phy->revision < PD_REV_30)
        return true;
    return cc_above(PORT_CVS_123);
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

static void send_request(const fe_target_t *t, req_origin_t origin)
{
    uint32_t rdo;
    if (t->type == PPS_PDO)
        rdo = pd_build_pps_rdo(t->pos, t->mv, t->ma);
    else
        rdo = pd_build_fixed_rdo(t->pos, t->ma, t->ma);

    pending = *t;
    pending_origin = origin;
    pd_phy_send(phy, MSG_TYPE_Request, 1, &rdo);
    set_state(FE_ST_WAIT_ACCEPT);
}

static void request_finished(bool ok)
{
    if (ok)
    {
        contract = pending;
        has_contract = true;
        keepalive_ts = millis();
    }
    if (pending_origin == ORIGIN_BRIDGE)
    {
        req_status = ok ? FE_REQ_OK : FE_REQ_FAIL;
    }
    set_state(FE_ST_READY);
}

static void do_hard_reset(void)
{
    pd_phy_send_hard_reset(phy);
    pd_phy_reset_protocol(phy);
    last_rx_id = 0xFF;
    has_contract = false;
    num_caps = 0;
    if (pending_origin == ORIGIN_BRIDGE && req_status == FE_REQ_BUSY)
        req_status = FE_REQ_FAIL;
    bridge_on_front_reset();
    set_state(FE_ST_WAIT_CAPS);
}

static void handle_source_caps(const pd_rx_msg_t *m, const pd_header_t *h)
{
    num_caps = h->num_objs;
    for (uint8_t i = 0; i < num_caps; i++)
    {
        caps[i] = pd_parse_pdo(pd_get_u32(&m->data[2 + i * 4]));
    }
    phy->revision = (h->revision < PD_REV_30) ? h->revision : PD_REV_30;

    /* 如有协议桥请求正在进行，被新的能力报文打断 */
    if (req_status == FE_REQ_BUSY)
        req_status = FE_REQ_FAIL;
    bridge_req_waiting = false;

    fe_target_t t = bridge_on_front_caps();
    send_request(&t, ORIGIN_CAPS);
}

static void send_sink_caps(void)
{
    uint32_t objs[2];
    objs[0] = pd_build_fixed_pdo(5000, 3000, 0);
    objs[1] = pd_build_pps_apdo(3300, 21000, 3000);
    pd_phy_send(phy, MSG_TYPE_Sink_Capabilities, 2, objs);
}

static void handle_msg(const pd_rx_msg_t *m)
{
    pd_header_t h = pd_parse_header(m->data[0] | (m->data[1] << 8));

    if (!h.extended && h.num_objs == 0 && h.msg_type == MSG_TYPE_Soft_Reset)
    {
        pd_phy_reset_protocol(phy);
        last_rx_id = 0xFF;
        send_ctrl(MSG_TYPE_Accept);
        if (req_status == FE_REQ_BUSY)
            req_status = FE_REQ_FAIL;
        set_state(FE_ST_WAIT_CAPS);
        return;
    }
    if (h.msg_id == last_rx_id)
        return;     /* 重发的报文 */
    last_rx_id = h.msg_id;

    if (h.extended)
    {
        send_not_supported();
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
            if ((pd_get_u32(&m->data[2]) >> 15) & 1)    /* 结构化 VDM */
                send_not_supported();
            break;
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
        break;
    case MSG_TYPE_Reject:
    case MSG_TYPE_Wait:
        if (state == FE_ST_WAIT_ACCEPT)
        {
            if (has_contract)
                request_finished(false);
            else
                do_hard_reset();
        }
        break;
    case MSG_TYPE_PS_RDY:
        if (state == FE_ST_WAIT_PS_RDY)
            request_finished(true);
        break;
    case MSG_TYPE_Get_Sink_Cap:
        send_sink_caps();
        break;
    case MSG_TYPE_GoodCRC:
    case MSG_TYPE_GotoMin:
    case MSG_TYPE_Ping:
    case MSG_TYPE_Not_Supported:
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
    phy->regs->PORT_CC1 &= ~(CC_PD | CC_PU_Mask);             /* CC3 不使用 */
    phy->regs->PORT_CC2 = (phy->regs->PORT_CC2 & ~CC_PU_Mask) | CC_PD; /* CC4R：Rd */
    pd_phy_set_cc(phy, 1);

    last_rx_id = 0xFF;
    has_contract = false;
    num_caps = 0;
    hard_reset_sent = false;
    set_state(FE_ST_WAIT_CAPS);
}

void fe_process(void)
{
    if (phy->hard_reset_rcvd)
    {
        phy->hard_reset_rcvd = false;
        pd_phy_reset_protocol(phy);
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
        if (!hard_reset_sent && elapsed > T_SINK_WAIT_CAP_MS)
        {
            hard_reset_sent = true;
            do_hard_reset();
        }
        else if (hard_reset_sent && elapsed > T_LEGACY_MS)
        {
            set_state(FE_ST_LEGACY);
            bridge_on_front_caps();
        }
        break;

    case FE_ST_WAIT_ACCEPT:
        if (elapsed > T_SENDER_RESPONSE_MS)
            do_hard_reset();
        break;

    case FE_ST_WAIT_PS_RDY:
        if (elapsed > T_PS_TRANSITION_MS)
            do_hard_reset();
        break;

    case FE_ST_READY:
        if (bridge_req_waiting)
        {
            if (sink_tx_ok() || millis() - bridge_req_ts > T_SINK_TX_WAIT_MS)
            {
                bridge_req_waiting = false;
                send_request(&pending, ORIGIN_BRIDGE);
            }
        }
        else if (has_contract && contract.type == PPS_PDO && millis() - keepalive_ts > PPS_KEEPALIVE_MS)
        {
            if (sink_tx_ok())
                send_request(&contract, ORIGIN_KEEPALIVE);
        }
        break;

    case FE_ST_LEGACY:
        break;
    }
}

bool fe_is_ready(void)
{
    return state == FE_ST_READY && has_contract && !bridge_req_waiting;
}

bool fe_is_legacy(void)
{
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
    *num = num_caps;
    return caps;
}

bool fe_caps_unconstrained(void)
{
    return num_caps > 0 && ((caps[0].raw >> 27) & 1);
}

const fe_target_t *fe_contract(void)
{
    return has_contract ? &contract : NULL;
}

bool fe_request(const fe_target_t *t)
{
    if (!fe_is_ready())
        return false;
    pending = *t;
    pending_origin = ORIGIN_BRIDGE;
    req_status = FE_REQ_BUSY;
    bridge_req_waiting = true;
    bridge_req_ts = millis();
    return true;
}

fe_req_status_t fe_request_status(void)
{
    return req_status;
}
