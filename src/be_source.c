#include "be_source.h"
#include "board.h"
#include "bridge.h"
#include "pd_phy.h"
#include "power_sw.h"
#include "timebase.h"

#define T_CC_POLL_MS            5
#define T_CC_DEBOUNCE_MS        150     /* tCCDebounce 100~200ms */
#define T_DETACH_POLL_MS        10
#define N_DETACH_DEBOUNCE       2       /* tPDDebounce 10~20ms */
#define T_VBUS_ON_SETTLE_MS     50      /* 打开 MOS 后等待 VBUS 建立，再发首条能力报文（tFirstSourceCap 250ms 内） */
#define T_SEND_CAPS_MS          150     /* tTypeCSendSourceCap 100~200ms */
#define N_CAPS_COUNT            50
#define T_SENDER_RESPONSE_MS    30
#define T_PS_HARD_RESET_MS      30      /* tPSHardReset 25~35ms */
#define T_SRC_RECOVER_MS        800     /* tSrcRecover 0.66~1s */
#define T_SINK_TX_MS            20      /* tSinkTx 16~20ms */
#define N_HARD_RESET_COUNT      2

/* 参考手册 §15.2.15 端口寄存器 */
#define PORT_CE                 (1u << 7)
#define PORT_CVS_Mask           (3u << 5)
#define PORT_CVS_022            (1u << 5)
#define PORT_CVS_066            (2u << 5)
#define PORT_RX_STATE_Mask      (7u << 2)

typedef enum
{
    BE_ST_UNATTACHED,
    BE_ST_ATTACH_WAIT,
    BE_ST_WAIT_VSAFE5V,
    BE_ST_STARTUP,
    BE_ST_SEND_CAPS,
    BE_ST_WAIT_REQUEST,
    BE_ST_TRANSITION,
    BE_ST_READY,
    BE_ST_SINK_TX_WAIT,
    BE_ST_HARD_RESET_WAIT,
    BE_ST_HARD_RESET_RECOVER,
    BE_ST_NO_PD,
} be_state_t;

typedef enum
{
    CC_OPEN,
    CC_RD,
    CC_RA,
} cc_state_t;

static pd_phy_t *const phy = &pd_phy_be;

static be_state_t state;
static uint32_t state_ts;
static uint32_t poll_ts;
static int8_t attach_cc;
static uint8_t open_count;
static uint8_t caps_count;
static uint8_t hard_reset_count;
static bool has_contract;
static bool caps_sent_once;
static uint8_t last_rx_id;

static void set_state(be_state_t s)
{
    state = s;
    state_ts = millis();
}

static void set_rp(uint8_t pu)
{
    USBPD_TypeDef *r = phy->regs;
    /* 接收期间保持比较器 CE=1、0.66V（WCH EVT） */
    r->PORT_CC1 = (r->PORT_CC1 & ~(CC_PU_Mask | CC_PD | PORT_CVS_Mask)) | PORT_CE | PORT_CVS_066 | pu;
    r->PORT_CC2 = (r->PORT_CC2 & ~(CC_PU_Mask | CC_PD | PORT_CVS_Mask)) | PORT_CE | PORT_CVS_066 | pu;
}

/*
 * 用 80uA 上拉判断 CC 状态（比较器最高只有 1.23V，330uA 下无法区分 Rd 与悬空）：
 *   Rd 5.1k：0.28~0.56V → 高于 0.22V、低于 0.66V
 *   Ra 1k  ：< 0.22V
 *   悬空   ：被拉到高电平
 */
static cc_state_t cc_sense(uint8_t cc_sel)
{
    volatile uint8_t *reg = pd_phy_port_reg(phy, cc_sel);
    uint8_t saved = *reg;

    *reg = (saved & ~(CC_PU_Mask | PORT_CVS_Mask)) | CC_PU_80 | PORT_CE | PORT_CVS_022;
    delay_us(20);
    bool above_022 = (*reg & CC_CMPO) != 0;
    *reg = (*reg & ~PORT_CVS_Mask) | PORT_CVS_066;
    delay_us(5);
    bool above_066 = (*reg & CC_CMPO) != 0;
    *reg = saved;

    if (!above_022)
        return CC_RA;
    if (!above_066)
        return CC_RD;
    return CC_OPEN;
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

static bool send_caps(void)
{
    uint32_t pdos[PD_MAX_DATA_OBJS];
    uint8_t n = bridge_back_caps(pdos);
    return pd_phy_send(phy, MSG_TYPE_Source_Capabilities, n, pdos);
}

static void go_unattached(void)
{
    power_sw_set(false);
    bridge_on_back_reset();
    set_rp(CC_PU_80);
    pd_phy_reset_protocol(phy);
    has_contract = false;
    hard_reset_count = 0;
    attach_cc = -1;
    set_state(BE_ST_UNATTACHED);
}

static void start_hard_reset(bool send)
{
    if (send)
        pd_phy_send_hard_reset(phy);
    hard_reset_count++;
    has_contract = false;
    set_state(BE_ST_HARD_RESET_WAIT);
}

static void handle_request(uint32_t rdo)
{
    if (bridge_eval_back_request(rdo))
    {
        send_ctrl(MSG_TYPE_Accept);
        bridge_start_transition();
        set_state(BE_ST_TRANSITION);
    }
    else
    {
        send_ctrl(MSG_TYPE_Reject);
        set_state(BE_ST_READY);
    }
}

static void handle_msg(const pd_rx_msg_t *m)
{
    pd_header_t h = pd_parse_header(m->data[0] | (m->data[1] << 8));

    if (!h.extended && h.num_objs == 0 && h.msg_type == MSG_TYPE_Soft_Reset)
    {
        pd_phy_reset_protocol(phy);
        last_rx_id = 0xFF;
        send_ctrl(MSG_TYPE_Accept);
        caps_count = 0;
        caps_sent_once = false;
        set_state(BE_ST_SEND_CAPS);
        return;
    }
    if (h.msg_id == last_rx_id)
        return;
    last_rx_id = h.msg_id;
    phy->revision = (h.revision < PD_REV_30) ? h.revision : PD_REV_30;

    if (h.extended)
    {
        send_not_supported();
        return;
    }

    if (h.num_objs > 0)
    {
        switch (h.msg_type)
        {
        case MSG_TYPE_Request:
            if (state == BE_ST_WAIT_REQUEST || state == BE_ST_READY)
                handle_request(pd_get_u32(&m->data[2]));
            break;
        case MSG_TYPE_Vendor_Defined:
            if ((pd_get_u32(&m->data[2]) >> 15) & 1)
                send_not_supported();
            break;
        case MSG_TYPE_Alert:
        case MSG_TYPE_BIST:
        case MSG_TYPE_Sink_Capabilities:
            break;
        default:
            send_not_supported();
            break;
        }
        return;
    }

    switch (h.msg_type)
    {
    case MSG_TYPE_Get_Source_Cap:
        if (state == BE_ST_READY || state == BE_ST_WAIT_REQUEST)
        {
            if (send_caps())
                set_state(BE_ST_WAIT_REQUEST);
        }
        break;
    case MSG_TYPE_GoodCRC:
    case MSG_TYPE_Accept:
    case MSG_TYPE_Reject:
    case MSG_TYPE_PS_RDY:
    case MSG_TYPE_Not_Supported:
    case MSG_TYPE_Ping:
    case MSG_TYPE_GotoMin:
        break;
    default:
        send_not_supported();
        break;
    }
}

static bool attached_state(void)
{
    return state != BE_ST_UNATTACHED && state != BE_ST_ATTACH_WAIT;
}

static bool pd_active_state(void)
{
    return state == BE_ST_SEND_CAPS || state == BE_ST_WAIT_REQUEST || state == BE_ST_TRANSITION ||
           state == BE_ST_READY || state == BE_ST_SINK_TX_WAIT || state == BE_ST_NO_PD;
}

static void poll_attach(void)
{
    cc_state_t s1 = cc_sense(0);
    cc_state_t s2 = cc_sense(1);
    int8_t cand = -1;
    if (s1 == CC_RD && s2 != CC_RD)
        cand = 0;
    else if (s2 == CC_RD && s1 != CC_RD)
        cand = 1;

    if (cand < 0)
    {
        attach_cc = -1;
        set_state(BE_ST_UNATTACHED);
        return;
    }
    if (state == BE_ST_UNATTACHED || cand != attach_cc)
    {
        attach_cc = cand;
        set_state(BE_ST_ATTACH_WAIT);
        return;
    }
    if (millis() - state_ts >= T_CC_DEBOUNCE_MS)
    {
        /* Attached.SRC：选定 CC，Rp 改为 3A（SinkTxOK） */
        pd_phy_set_cc(phy, attach_cc);
        pd_phy_reset_protocol(phy);
        phy->hard_reset_rcvd = false;
        last_rx_id = 0xFF;
        set_rp(CC_PU_330);
        open_count = 0;
        hard_reset_count = 0;
        set_state(BE_ST_WAIT_VSAFE5V);
    }
}

static bool poll_detach(void)
{
    if (phy->regs->CONTROL & PORT_RX_STATE_Mask)
        return false;   /* 正在接收，下次再查 */
    if (cc_sense(attach_cc) == CC_OPEN)
    {
        if (++open_count >= N_DETACH_DEBOUNCE)
        {
            go_unattached();
            return true;
        }
    }
    else
    {
        open_count = 0;
    }
    return false;
}

void be_init(void)
{
    pd_phy_init(phy, USBPD0, USBPD0_IRQn, 1 /* Source */, 1 /* DFP */);
    go_unattached();
}

void be_process(void)
{
    uint32_t now = millis();

    if (!attached_state())
    {
        if (now - poll_ts >= T_CC_POLL_MS)
        {
            poll_ts = now;
            poll_attach();
        }
        return;
    }

    if (now - poll_ts >= T_DETACH_POLL_MS)
    {
        poll_ts = now;
        if (poll_detach())
            return;
    }

    if (phy->hard_reset_rcvd)
    {
        phy->hard_reset_rcvd = false;
        start_hard_reset(false);
    }

    pd_rx_msg_t m;
    while (pd_phy_rx_pop(phy, &m))
    {
        if (pd_active_state())
            handle_msg(&m);
    }

    if (pd_active_state() && state != BE_ST_NO_PD && bridge_take_back_hard_reset())
    {
        start_hard_reset(true);
    }

    uint32_t elapsed = millis() - state_ts;
    switch (state)
    {
    case BE_ST_WAIT_VSAFE5V:
        if (bridge_front_ready_for_vsafe5v())
        {
            power_sw_set(true);
            set_state(BE_ST_STARTUP);
        }
        break;

    case BE_ST_STARTUP:
        if (elapsed >= T_VBUS_ON_SETTLE_MS)
        {
            caps_count = 0;
            caps_sent_once = false;
            set_state(BE_ST_SEND_CAPS);
        }
        break;

    case BE_ST_SEND_CAPS:
        if (!caps_sent_once || elapsed >= T_SEND_CAPS_MS)
        {
            caps_sent_once = true;
            if (send_caps())
            {
                set_state(BE_ST_WAIT_REQUEST);
            }
            else if (++caps_count > N_CAPS_COUNT)
            {
                set_state(BE_ST_NO_PD);   /* 非 PD 设备：保持 5V 供电 */
            }
            else
            {
                set_state(BE_ST_SEND_CAPS);
            }
        }
        break;

    case BE_ST_WAIT_REQUEST:
        if (elapsed > T_SENDER_RESPONSE_MS)
            start_hard_reset(true);
        break;

    case BE_ST_TRANSITION:
        switch (bridge_poll_transition())
        {
        case BRIDGE_OK:
            send_ctrl(MSG_TYPE_PS_RDY);
            has_contract = true;
            hard_reset_count = 0;
            bridge_on_back_contract();
            set_rp(CC_PU_330);
            set_state(BE_ST_READY);
            break;
        case BRIDGE_FAIL:
            start_hard_reset(true);
            break;
        default:
            break;
        }
        break;

    case BE_ST_READY:
        if (bridge_take_back_caps_dirty())
        {
            /* 源端主动发起 AMS：先置 SinkTxNG（Rp 1.5A），等待 tSinkTx */
            set_rp(CC_PU_180);
            set_state(BE_ST_SINK_TX_WAIT);
        }
        break;

    case BE_ST_SINK_TX_WAIT:
        if (elapsed >= T_SINK_TX_MS)
        {
            bool ok = send_caps();
            set_rp(CC_PU_330);
            if (ok)
                set_state(BE_ST_WAIT_REQUEST);
            else
                start_hard_reset(true);
        }
        break;

    case BE_ST_HARD_RESET_WAIT:
        if (elapsed >= T_PS_HARD_RESET_MS)
        {
            power_sw_set(false);
            bridge_on_back_reset();
            set_state(BE_ST_HARD_RESET_RECOVER);
        }
        break;

    case BE_ST_HARD_RESET_RECOVER:
        if (elapsed >= T_SRC_RECOVER_MS)
        {
            pd_phy_reset_protocol(phy);
            last_rx_id = 0xFF;
            set_rp(CC_PU_330);
            if (hard_reset_count > N_HARD_RESET_COUNT)
                set_state(BE_ST_NO_PD);
            else
                set_state(BE_ST_WAIT_VSAFE5V);
        }
        break;

    case BE_ST_NO_PD:
        if (!power_sw_is_on() && bridge_front_ready_for_vsafe5v())
            power_sw_set(true);
        break;

    default:
        break;
    }
}
