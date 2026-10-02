#include "be_source.h"
#include "analog.h"
#include "board.h"
#include "bridge.h"
#include "cfg.h"
#include "evlog.h"
#include "pdinfo.h"
#include "pd_phy.h"
#include "power_sw.h"
#include "timebase.h"

#define T_CC_POLL_MS            5
#define T_CC_DEBOUNCE_MS        150     /* tCCDebounce 100~200ms */
#define T_DETACH_POLL_MS        10
#define N_DETACH_DEBOUNCE       2       /* tPDDebounce 10~20ms */
#define N_DETACH_RX_SKIP        2       /* 接收忙最多推迟 2 次检测，防止 RX_STATE 卡住导致永远检测不到拔出 */
#define T_DETACH_DISCHARGE_MS   650     /* tVBUSOFF：拔出后先把前端降到 5V 再关 MOS */
#define VBUS_DISCHARGED_MV      5750    /* 前端 VBUS 已回到 vSafe5V 附近（含 ADC 误差） */
#define T_VBUS_ON_SETTLE_MS     50      /* 打开 MOS 后等待 VBUS 建立，再发首条能力报文（tFirstSourceCap 250ms 内） */
#define T_SELFQA_MS             300     /* 虚拟 E-Marker：Source_Capabilities 后无 SOP' 查询则自问自答 */
#define T_SEND_CAPS_MS          150     /* tTypeCSendSourceCap 100~200ms */
#define N_CAPS_COUNT            50
#define T_SENDER_RESPONSE_MS    30
#define T_PS_HARD_RESET_MS      30      /* tPSHardReset 25~35ms */
#define T_SRC_RECOVER_MS        800     /* tSrcRecover 0.66~1s */
#define T_SINK_TX_MS            20      /* tSinkTx 16~20ms */
#define N_HARD_RESET_COUNT      2
#define T_QUERY_DELAY_MS        200     /* 合约建立后稍等再查询设备（Lab 二次握手、信息透传） */
#define T_QUERY_GAP_MS          30      /* 连续查询之间的间隔 */
#define T_CABLE_RETRY_MS        100     /* 线材 Discover Identity 重试间隔（BUSY 或无应答） */
#define N_CABLE_TRIES           4       /* 线材无应答 / BUSY 的最多尝试次数（无 VCONN 时线材不会回应，不必久等） */
#define PI_Q_CABLE              0x80    /* 本地查询项：向线材（SOP'）Discover Identity，与 pdinfo 的 PI_Q_* 不重叠 */

/* 参考手册 §15.2.15 端口寄存器 */
#define PORT_CE                 (1u << 7)
#define PORT_CVS_Mask           (3u << 5)
#define PORT_CVS_022            (1u << 5)
#define PORT_CVS_066            (2u << 5)
#define PORT_CVS_123            (3u << 5)
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
    BE_ST_DETACH_DISCHARGE,
    BE_ST_QUERY_TX_WAIT,        /* SinkTxNG 后查询设备（Lab 二次握手、信息透传） */
    BE_ST_QUERY_WAIT,
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
static uint8_t rx_skip_count;
static uint8_t caps_count;
static uint8_t hard_reset_count;
static bool has_contract;
static bool caps_sent_once;
static bool selfqa_armed;               /* 虚拟 E-Marker：Source_Capabilities 已发出，等待 Sink 的 SOP' 查询 */
static bool selfqa_done;                /* 本次连接已有 SOP' 查询（或已自问自答），不再触发 */
static uint32_t selfqa_ts;
static uint8_t selfqa_cable_id;         /* 自问自答中"线材"一侧的 MessageID */
static uint8_t last_rx_id;
static uint8_t query;               /* 进行中的查询 PI_Q_*，0 = 无 */
static uint16_t query_gap;          /* READY 后多久可以发下一项查询 */

/* 后端线材 E-Marker：每次连接都向线材发 SOP' Discover Identity（不看 Ra：单 CC 焊盘没有另一根 CC）；E-Marker 需要 VCONN 供电才会应答 */
static uint8_t cable_status;        /* be_cable_status_t */

static bool cable_virtual(void)
{
    return (cfg_ext()->flags & CFGX_CABLE) != 0;
}
static uint8_t cable_tries;
static bool cable_ra;               /* 另一根 CC 脚出现 Ra（仅供显示） */
static uint8_t cable_n;             /* 已读到的 VDO 数 */
static uint32_t cable_vdo[BE_CABLE_VDOS];
static uint8_t cable_last_rx_id;

/*
 * VCONN：原版硬件没有 VCONN 开关，改用 MCU 引脚直接驱动线材的 VCONN 脚：另一根 CC 脚（PA0/PA1）推挽输出 3.3V。
 * 只在检测到 Ra（线材带 E-Marker）时供电。3.3V 低于规范的 4.75V，也受引脚驱动能力限制（Ra 1k 约 3.3mA + E-Marker 电流），
 * 但无源 E-Marker 多数能在 3.0V 以上工作；读不到时才需要硬件 VCONN 开关（5V），那时覆盖这两个弱函数。
 */
static uint16_t vconn_pin;                  /* 正在供电的 CC 引脚，0 = 未供电 */

__attribute__((weak)) void be_vconn_set(bool on)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    if (vconn_pin)
    {
        gpio.GPIO_Pin = vconn_pin;
        gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_Init(GPIOA, &gpio);
        vconn_pin = 0;
    }
    if (on && cable_ra)
    {
        vconn_pin = attach_cc == 0 ? GPIO_Pin_1 : GPIO_Pin_0;     /* 与通信的 CC 相反的那根 */
        GPIOA->BSHR = vconn_pin;
        gpio.GPIO_Pin = vconn_pin;
        gpio.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_Init(GPIOA, &gpio);
    }
}
__attribute__((weak)) bool be_vconn_available(void)
{
    return true;                            /* 3.3V 引脚供电（见上） */
}

static void cable_clear(void)
{
    cable_status = CABLE_NONE;
    cable_ra = false;
    cable_tries = 0;
    cable_n = 0;
    cable_last_rx_id = 0xFF;
    selfqa_armed = false;
    selfqa_done = false;
    selfqa_cable_id = 0;
    phy->sop1_en = false;
    be_vconn_set(false);
}

uint8_t be_cable_status(void)
{
    return cable_status;
}

bool be_cable_ra(void)
{
    return cable_ra;
}

/* 虚拟 E-Marker 的 VDO 个数：有源线材（ID Header 产品类型 = 4）多一个 Cable VDO2 */
static uint8_t virtual_cable_n(void)
{
    return cfg_cable_n(cfg_ext()->cable[0]);
}

uint8_t be_cable_vdos(uint32_t *out)
{
    if (cable_virtual())
    {
        memcpy(out, cfg_ext()->cable, virtual_cable_n() * 4);
        return virtual_cable_n();
    }
    memcpy(out, cable_vdo, cable_n * 4);
    return cable_n;
}

/* Cable VDO1（第 4 个 VDO）bit6:5 = VBUS 电流能力：01 = 3A，10 = 5A；产品类型须是无源 / 有源线材（ID Header bit29:27 = 3 / 4） */
bool be_cable_5a(void)
{
    if (cable_virtual())
        return cfg_ext_cable_5a(cfg_ext());
    uint8_t type = (cable_vdo[0] >> 27) & 7;
    return cable_status == CABLE_OK && cable_n >= 4 && (type == 3 || type == 4) && ((cable_vdo[3] >> 5) & 3) == 2;
}

/* 记录线材读取结果（开启记录时写入 Flash） */
static void log_cable(void)
{
    ev_cable_t e = {cable_status, (uint8_t)((cable_ra ? 1 : 0) | (be_vconn_available() ? 2 : 0)), cable_n, {0}};
    memcpy(e.vdo, cable_vdo, cable_n * 4);
    evlog_add(EV_CABLE, &e, 3 + cable_n * 4);
}

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
 *   Rd 5.1k        ：0.28~0.56V
 *   Rd 电压钳位型  ：约 0.9~1.3V（无电设备的 dead-battery Rd，如电量耗尽的 iPhone 实测 0.9V）
 *   Ra 1k          ：< 0.22V
 *   悬空           ：被拉到 VDD33
 * 规范 Default USB Rp 下 vRd 上限 1.6V；比较器最高档 1.23V，因此以 1.23V 作为 Rd/悬空分界。
 */
static cc_state_t cc_sense(uint8_t cc_sel)
{
    volatile uint8_t *reg = pd_phy_port_reg(phy, cc_sel);
    uint8_t saved = *reg;

    *reg = (saved & ~(CC_PU_Mask | PORT_CVS_Mask)) | CC_PU_80 | PORT_CE | PORT_CVS_022;
    delay_us(20);
    bool above_022 = (*reg & CC_CMPO) != 0;
    *reg = (*reg & ~PORT_CVS_Mask) | PORT_CVS_123;
    delay_us(10);
    bool above_123 = (*reg & CC_CMPO) != 0;
    *reg = saved;

    if (!above_022)
        return CC_RA;
    if (!above_123)
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

static void log_reset(uint8_t kind)
{
    ev_reset_t e = {1, kind, (uint8_t)state, phy->dbg_rx_reset};
    evlog_add(EV_RESET, &e, sizeof(e));
}

static void go_unattached(void)
{
    cable_clear();
    power_sw_set(false);
    pdinfo_dev_reset();
    pdinfo_dev_attached(false);
    bridge_on_back_reset();
    bridge_on_back_detach();
    set_rp(CC_PU_80);
    pd_phy_reset_protocol(phy);
    has_contract = false;
    hard_reset_count = 0;
    attach_cc = -1;
    set_state(BE_ST_UNATTACHED);
}

/*
 * 设备拔出：后端 VBUS 没有放电电路（DSCG 未接），直接关 MOS 会把高压留在母座上。
 * 先保持 MOS 导通、让前端回到 5V（充电器负责放电），前端 VBUS 回到 vSafe5V 或超时后再关断。
 */
static void on_detach(void)
{
    evlog_add(EV_BE_DETACH, NULL, 0);
    cable_clear();
    pdinfo_dev_reset();
    pdinfo_dev_attached(false);
    bridge_on_back_reset();
    set_rp(CC_PU_80);
    pd_phy_reset_protocol(phy);
    has_contract = false;
    attach_cc = -1;
    if (power_sw_is_on())
        set_state(BE_ST_DETACH_DISCHARGE);
    else
        go_unattached();
}

static void start_hard_reset(bool send)
{
    log_reset(send ? RST_HARD_SENT : RST_HARD_RCVD);
    if (send)
        pd_phy_send_hard_reset(phy);
    hard_reset_count++;
    has_contract = false;
    pdinfo_dev_reset();
    set_state(BE_ST_HARD_RESET_WAIT);
}

/* 下一项查询：Lab 二次握手优先，其次信息透传 */
static uint8_t next_query(void)
{
    if (cable_status == CABLE_PENDING && cable_tries < N_CABLE_TRIES && phy->revision >= PD_REV_30)
        return PI_Q_CABLE;
    if (bridge_rear_query_wanted())
        return PI_Q_EXTCAPS;
    return pdinfo_dev_next_query();
}

static bool send_query(uint8_t q)
{
    static const uint8_t battery_ref[1] = {0};
    uint32_t vdm = VDM_DISC_IDENT_REQ;
    switch (q)
    {
    case PI_Q_CABLE:
        return pd_phy_send_sop1(phy, MSG_TYPE_Vendor_Defined, 1, &vdm, PD_REV_30);
    case PI_Q_IDENT:
        return pd_phy_send(phy, MSG_TYPE_Vendor_Defined, 1, &vdm);
    case PI_Q_EXTCAPS:
        return pd_phy_send(phy, MSG_TYPE_Get_Sink_Cap_Extended, 0, NULL);
    case PI_Q_BCAP:
        return pd_phy_send_ext(phy, MSG_TYPE_Get_Battery_Cap, battery_ref, 1);
    default:
        return pd_phy_send_ext(phy, MSG_TYPE_Get_Battery_Status, battery_ref, 1);
    }
}

/* 查询结束（数据已交给 pdinfo / 协议桥，或设备拒绝、超时） */
static void query_end(void)
{
    if (query == PI_Q_CABLE)
    {
        /* 线材无应答 / BUSY / NAK：BUSY 与无应答重试，次数用完记为无应答 */
        if (cable_status == CABLE_PENDING && ++cable_tries >= N_CABLE_TRIES)
        {
            cable_status = CABLE_NO_REPLY;
            log_cable();
        }
        query = 0;
        query_gap = T_CABLE_RETRY_MS;
        set_state(BE_ST_READY);
        return;
    }
    if (query == PI_Q_EXTCAPS && bridge_rear_query_wanted())
        bridge_on_rear_sink_modes(-1);
    pdinfo_dev_query_done(query);
    query = 0;
    query_gap = T_QUERY_GAP_MS;
    set_state(BE_ST_READY);
}

/* 代答设备的扩展查询：数据来自前端读到的充电器信息 */
static void answer_ext_query(uint8_t type, const uint8_t *d, uint8_t size)
{
    uint8_t buf[PI_EXT_MAX];
    uint8_t n = 0;
    if (type == MSG_TYPE_Get_Manufacturer_Info && size >= 1 && d[0] == 0)     /* 目标 = 端口 */
        n = pdinfo_be_midb(buf);
    if (n)
        pd_phy_send_ext(phy, MSG_TYPE_Manufacturer_Info, buf, n);
    else
        send_not_supported();
}

static void handle_vdm(const pd_rx_msg_t *m, uint8_t n)
{
    uint32_t vdm = pd_get_u32(&m->data[2]);
    if (!VDM_STRUCTURED(vdm))
        return;
    if (VDM_CMD_TYPE(vdm) != 0)
    {
        /* 设备对 Discover Identity 的应答 */
        if (state == BE_ST_QUERY_WAIT && query == PI_Q_IDENT && VDM_IS_DISC_IDENT(vdm))
        {
            if (VDM_CMD_TYPE(vdm) == VDM_ACK && n > 1)
            {
                uint32_t vdo[PI_MAX_VDOS];
                uint8_t k = n - 1 > PI_MAX_VDOS ? PI_MAX_VDOS : n - 1;
                for (uint8_t i = 0; i < k; i++)
                    vdo[i] = pd_get_u32(&m->data[6 + 4 * i]);
                pdinfo_dev_ident(vdo, k);
            }
            if (VDM_CMD_TYPE(vdm) != VDM_BUSY)
                query_end();
        }
        return;
    }
    /* 设备查询本端身份：身份透传时回充电器的 */
    pi_ident_t id;
    if (VDM_IS_DISC_IDENT(vdm) && pdinfo_be_ident(&id))
    {
        uint32_t o[1 + PI_MAX_VDOS];
        o[0] = VDM_REPLY(vdm, VDM_ACK);
        memcpy(&o[1], id.vdo, id.n * 4);
        pd_phy_send(phy, MSG_TYPE_Vendor_Defined, 1 + id.n, o);
        return;
    }
    send_not_supported();
}

/* 虚拟 E-Marker：设备（端口，bit8 = 0）发来的 SOP' 请求，按自订的 VDO 应答 */
static void virtual_cable_reply(const pd_rx_msg_t *m, const pd_header_t *h)
{
    uint8_t rev = h->revision < PD_REV_30 ? h->revision : PD_REV_30;
    phy->sop1_plug = 1;                 /* 以线材身份发送 */
    if (!h->extended && h->num_objs == 0 && h->msg_type == MSG_TYPE_Soft_Reset)
    {
        cable_last_rx_id = 0xFF;
        phy->tx_msg_id_sop1 = 0;
        pd_phy_send_sop1(phy, MSG_TYPE_Accept, 0, NULL, rev);
    }
    else if (h->msg_id != cable_last_rx_id)
    {
        cable_last_rx_id = h->msg_id;
        uint32_t vdm = pd_get_u32(&m->data[2]);
        if (!h->extended && h->msg_type == MSG_TYPE_Vendor_Defined && h->num_objs > 0 && VDM_STRUCTURED(vdm) &&
            VDM_CMD_TYPE(vdm) == 0)
        {
            if (VDM_IS_DISC_IDENT(vdm))
            {
                uint32_t o[6] = {VDM_REPLY(vdm, VDM_ACK)};
                memcpy(&o[1], cfg_ext()->cable, sizeof(cfg_ext()->cable));
                pd_phy_send_sop1(phy, MSG_TYPE_Vendor_Defined, 1 + virtual_cable_n(), o, rev);
            }
            else
            {
                uint32_t nak = VDM_REPLY(vdm, VDM_NAK);
                pd_phy_send_sop1(phy, MSG_TYPE_Vendor_Defined, 1, &nak, rev);
            }
        }
    }
    phy->sop1_plug = 0;
}

/* 虚拟 E-Marker 自问自答：Sink 在 Source_Capabilities 后 T_SELFQA_MS 内没有查询线材时，
 * 由本端同时扮演端口与线材，在 CC 上完整走一遍 Discover Identity（供抓包仪器读取线材信息）：
 * 端口 Discover Identity → 线材 GoodCRC → 线材 ACK → 端口 GoodCRC */
static void selfqa_run(void)
{
    uint8_t pid = phy->tx_msg_id_sop1;
    uint8_t cid = selfqa_cable_id;
    uint32_t req = VDM_DISC_IDENT_REQ;
    uint32_t o[6] = {VDM_REPLY(req, VDM_ACK)};
    memcpy(&o[1], cfg_ext()->cable, sizeof(cfg_ext()->cable));

    pd_phy_send_sop1_raw(phy, pd_build_header(MSG_TYPE_Vendor_Defined, 1, pid, 0, 0, PD_REV_30), &req, 1);
    delay_us(30);
    pd_phy_send_sop1_raw(phy, pd_build_header(MSG_TYPE_GoodCRC, 0, pid, 1, 0, PD_REV_30), NULL, 0);
    delay_ms(1);
    pd_phy_send_sop1_raw(phy, pd_build_header(MSG_TYPE_Vendor_Defined, 1 + virtual_cable_n(), cid, 1, 0, PD_REV_30), o,
                         1 + virtual_cable_n());
    delay_us(30);
    pd_phy_send_sop1_raw(phy, pd_build_header(MSG_TYPE_GoodCRC, 0, cid, 0, 0, PD_REV_30), NULL, 0);
    phy->tx_msg_id_sop1 = (pid + 1) & 0x7;
    selfqa_cable_id = (cid + 1) & 0x7;
}

/* 线材（SOP'）的应答：Discover Identity */
static void handle_cable_msg(const pd_rx_msg_t *m)
{
    pd_header_t h = pd_parse_header(m->data[0] | (m->data[1] << 8));
    if (!h.power_role)                  /* 端口发来的（不是线材的应答） */
    {
        if (cable_status == CABLE_VIRTUAL)
        {
            selfqa_done = true;         /* Sink 自己来查了 */
            virtual_cable_reply(m, &h);
        }
        return;
    }
    if (h.msg_id == cable_last_rx_id)
        return;
    cable_last_rx_id = h.msg_id;
    if (h.extended || h.msg_type != MSG_TYPE_Vendor_Defined || h.num_objs == 0)
        return;
    uint32_t vdm = pd_get_u32(&m->data[2]);
    if (!VDM_STRUCTURED(vdm) || VDM_CMD_TYPE(vdm) == 0 || !VDM_IS_DISC_IDENT(vdm))
        return;
    if (state != BE_ST_QUERY_WAIT || query != PI_Q_CABLE)
        return;
    uint8_t cmd = VDM_CMD_TYPE(vdm);
    if (cmd == VDM_ACK && h.num_objs > 1)
    {
        uint8_t k = h.num_objs - 1 > BE_CABLE_VDOS ? BE_CABLE_VDOS : h.num_objs - 1;
        for (uint8_t i = 0; i < k; i++)
            cable_vdo[i] = pd_get_u32(&m->data[6 + 4 * i]);
        cable_n = k;
        cable_status = CABLE_OK;
        log_cable();
        bridge_on_back_cable(be_cable_5a());
    }
    else if (cmd == VDM_NAK)
    {
        cable_status = CABLE_NAK;
        log_cable();
    }
    query_end();
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
        log_reset(RST_SOFT_RCVD);
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
        uint16_t ext = m->data[2] | (m->data[3] << 8);
        const uint8_t *d = &m->data[4];
        uint8_t size = m->len > 4 ? m->len - 4 : 0;
        if (EXT_DATA_SIZE(ext) < size)
            size = EXT_DATA_SIZE(ext);
        if (ext & EXT_REQUEST_CHUNK)
            return;
        switch (h.msg_type)
        {
        case MSG_TYPE_Sink_Capabilities_Extended:
            if (state == BE_ST_QUERY_WAIT && query == PI_Q_EXTCAPS)
            {
                pdinfo_dev_skedb(d, size);
                if (bridge_rear_query_wanted())
                    bridge_on_rear_sink_modes(size > SKEDB_SINK_MODES ? d[SKEDB_SINK_MODES] : -1);
                query_end();
            }
            break;
        case MSG_TYPE_Battery_Capabilities:
            if (state == BE_ST_QUERY_WAIT && query == PI_Q_BCAP)
            {
                pdinfo_dev_bcap(d, size);
                query_end();
            }
            break;
        case MSG_TYPE_Get_Manufacturer_Info:
            answer_ext_query(h.msg_type, d, size);
            break;
        default:
            send_not_supported();
            break;
        }
        return;
    }

    if (h.num_objs > 0)
    {
        switch (h.msg_type)
        {
        case MSG_TYPE_Request:
            if (state == BE_ST_QUERY_WAIT)
                query_end();    /* 设备抢先发了请求：本次查询作废 */
            if (state == BE_ST_WAIT_REQUEST || state == BE_ST_READY)
                handle_request(pd_get_u32(&m->data[2]));
            break;
        case MSG_TYPE_Vendor_Defined:
            handle_vdm(m, h.num_objs);
            break;
        case MSG_TYPE_Battery_Status:
            if (state == BE_ST_QUERY_WAIT && query == PI_Q_BSTAT)
            {
                pdinfo_dev_bsdo(pd_get_u32(&m->data[2]));
                query_end();
            }
            break;
        case MSG_TYPE_Alert:
            pdinfo_dev_alert(pd_get_u32(&m->data[2]));
            break;
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
    case MSG_TYPE_Get_Source_Cap_Extended:
    {
        uint8_t buf[PI_EXT_MAX];
        uint8_t n = pdinfo_be_scedb(buf);
        if (n)
            pd_phy_send_ext(phy, MSG_TYPE_Source_Capabilities_Extended, buf, n);
        else
            send_not_supported();
        break;
    }
    case MSG_TYPE_Get_Source_Info:
    {
        uint32_t sido;
        if (pdinfo_be_sido(&sido))
            pd_phy_send(phy, MSG_TYPE_Source_Info, 1, &sido);
        else
            send_not_supported();
        break;
    }
    case MSG_TYPE_Get_Status:
    {
        uint8_t buf[PI_EXT_MAX];
        uint8_t n = pdinfo_be_status(buf);
        if (n)
            pd_phy_send_ext(phy, MSG_TYPE_Status, buf, n);
        else
            send_not_supported();
        break;
    }
    case MSG_TYPE_Reject:
    case MSG_TYPE_Not_Supported:
        if (state == BE_ST_QUERY_WAIT)
            query_end();
        break;
    case MSG_TYPE_GoodCRC:
    case MSG_TYPE_Accept:
    case MSG_TYPE_PS_RDY:
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
           state == BE_ST_READY || state == BE_ST_SINK_TX_WAIT || state == BE_ST_NO_PD ||
           state == BE_ST_QUERY_TX_WAIT || state == BE_ST_QUERY_WAIT;
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
        rx_skip_count = 0;
        hard_reset_count = 0;
        cable_clear();
        cable_ra = (attach_cc == 0 ? s2 : s1) == CC_RA;
        cable_status = cable_virtual() ? CABLE_VIRTUAL : CABLE_PENDING;     /* 不论有无 Ra，都向线材发 Discover Identity；策略为 5A 时不读线材 */
        phy->tx_msg_id_sop1 = 0;
        phy->sop1_en = true;
        uint8_t cc = (uint8_t)attach_cc;
        evlog_add(EV_BE_ATTACH, &cc, 1);
        pdinfo_dev_attached(true);
        set_state(BE_ST_WAIT_VSAFE5V);
    }
}

static bool poll_detach(void)
{
    if ((phy->regs->CONTROL & PORT_RX_STATE_Mask) && rx_skip_count < N_DETACH_RX_SKIP)
    {
        rx_skip_count++;
        return false;   /* 正在接收，下次再查 */
    }
    rx_skip_count = 0;
    if (cc_sense(attach_cc) == CC_OPEN)
    {
        if (++open_count >= N_DETACH_DEBOUNCE)
        {
            on_detach();
            return true;
        }
    }
    else
    {
        open_count = 0;
    }
    return false;
}

bool be_flash_safe(void)
{
    return state == BE_ST_UNATTACHED || state == BE_ST_ATTACH_WAIT || state == BE_ST_READY || state == BE_ST_NO_PD;
}

uint8_t be_state_code(void)
{
    return (uint8_t)state;
}

bool be_attached(void)
{
    return attached_state();
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

    /* 线材策略在连接期间被修改：切换虚拟 E-Marker / 读取 */
    if (cable_virtual() != (cable_status == CABLE_VIRTUAL) && (cable_status == CABLE_VIRTUAL || cable_status == CABLE_PENDING ||
                                                                 cable_status == CABLE_NO_REPLY || cable_status == CABLE_NAK))
    {
        cable_status = cable_virtual() ? CABLE_VIRTUAL : CABLE_PENDING;
        cable_tries = 0;
    }

    if (state == BE_ST_DETACH_DISCHARGE)
    {
        bool front_5v = bridge_front_ready_for_vsafe5v() && analog_vbus_mv() < VBUS_DISCHARGED_MV;
        if (front_5v || now - state_ts >= T_DETACH_DISCHARGE_MS)
            go_unattached();
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
        {
            if (m.sop)
                handle_cable_msg(&m);
            else
                handle_msg(&m);
        }
    }

    if (pd_active_state() && state != BE_ST_NO_PD && bridge_take_back_hard_reset())
    {
        start_hard_reset(true);
    }

    uint32_t elapsed = millis() - state_ts;

    if (selfqa_armed && (state == BE_ST_WAIT_REQUEST || state == BE_ST_READY) && millis() - selfqa_ts >= T_SELFQA_MS)
    {
        selfqa_armed = false;
        selfqa_done = true;
        selfqa_run();
    }
    switch (state)
    {
    case BE_ST_WAIT_VSAFE5V:
        if (bridge_front_ready_for_vsafe5v())
        {
            power_sw_set(true);
            if (!cable_virtual())
                be_vconn_set(true);     /* VBUS 之后供 VCONN（单 CC 焊盘没有 Ra 可查，硬件自行决定），tVCONNStable 内线材上电 */
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
                if (cable_status == CABLE_VIRTUAL && !selfqa_armed && !selfqa_done)
                {
                    selfqa_armed = true;
                    selfqa_ts = millis();
                }
                set_state(BE_ST_WAIT_REQUEST);
            }
            else if (++caps_count > N_CAPS_COUNT)
            {
                pdinfo_dev_attached(false);
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
            pdinfo_dev_start(phy->revision >= PD_REV_30);
            /* 信息透传：尽快读设备身份（前端新合约后充电器很快会来查询）；Lab 二次握手仍稍等 */
            query_gap = bridge_rear_query_wanted() ? T_QUERY_DELAY_MS : T_QUERY_GAP_MS;
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
        else if (elapsed >= query_gap && (query = next_query()) != 0)
        {
            if (phy->revision < PD_REV_30)
            {
                query_end();    /* PD2.0 没有这些报文，也不支持 APDO */
            }
            else
            {
                set_rp(CC_PU_180);
                set_state(BE_ST_QUERY_TX_WAIT);
            }
        }
        break;

    case BE_ST_QUERY_TX_WAIT:
        if (elapsed >= T_SINK_TX_MS)
        {
            bool ok = send_query(query);
            set_rp(CC_PU_330);
            if (ok)
                set_state(BE_ST_QUERY_WAIT);
            else
                query_end();
        }
        break;

    case BE_ST_QUERY_WAIT:
        if (elapsed > (query == PI_Q_CABLE ? T_SENDER_RESPONSE_MS + 10 : T_SENDER_RESPONSE_MS))
            query_end();
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
            {
                pdinfo_dev_attached(false);
                set_state(BE_ST_NO_PD);
            }
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
