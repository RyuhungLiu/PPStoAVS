#include "ufcs.h"
#include "analog.h"
#include "board.h"
#include "evlog.h"
#include "timebase.h"
#include "ufcs_codec.h"
#include "ufcs_phy.h"

/* 规范表 27 / 6.4.7 */
#define T_ACK_RECEIVE_MS        10      /* tACKReceive */
#define T_RESPONSE_MS           50      /* tSenderResponse（tReceiverResponse 40ms） */
#define T_POWER_SUPPLY_MS       550     /* tPowerSupply */
#define T_ACK_TX_US             150     /* tACKtransmit ≥ 100µs */
#define T_MSG_GAP_US            2200    /* 我方两包之间 ≥ 2ms（tMsgTransDelay） */
#define N_MSG_RETRY             3
#define T_POLL_MS               400     /* 供电设备看门狗默认 1s，需要更频繁的消息 */
#define T_SETTLE_MS             200     /* Power_Ready 之后等电压稳定再测 */
#define MAX_TARGETS             10
#define LOG_ACKS                4       /* 只记前几个 ACK 原始报文，避免占满记录 */
#define LOG_INFOS               8

typedef enum { F_IDLE, F_PING, F_CAPS, F_REQ, F_SETTLE, F_HOLD, F_DONE } flow_t;
typedef enum { W_NONE, W_CAPS, W_ACCEPT, W_READY, W_INFO } wait_t;
typedef enum { TX_NONE, TX_QUEUED, TX_SENDING, TX_WAIT_ACK } txs_t;

typedef struct
{
    uint8_t  mode;
    uint16_t mv;
    uint16_t ma;
} target_t;

static struct
{
    bool     up;
    flow_t   flow;
    wait_t   wait;
    uint32_t wait_deadline;

    uint8_t  tx_addr;               /* 我方发出消息头里的「接收方地址」 */
    uint8_t  tx_baud;
    uint8_t  num;                   /* MsgNumberCounter */

    /* 正在进行的消息（等 ACK） */
    txs_t    txs;
    ufcs_msg_t msg;
    uint8_t  raw[UFCS_MAX_PKT];
    uint8_t  rawn;
    uint8_t  retry;
    bool     nck;
    uint32_t ack_deadline_us;

    /* 待发 ACK/NCK */
    bool     ack_pending;
    uint8_t  ack_cmd;
    uint8_t  ack_num;
    uint32_t ack_at_us;

    uint32_t last_act_us;           /* 最近一次收发结束时间，用于包间隔 */
    uint32_t t_hs_ms;
    uint32_t t_req_ms;
    uint32_t t_accept_ms;
    uint32_t last_poll_ms;

    ufcs_mode_t modes[UFCS_MAX_MODES];
    uint8_t  nmodes;
    target_t targets[MAX_TARGETS];
    uint8_t  ntargets, ti;
    uint32_t settle_at_ms, t_ready_ms;
    uint8_t  info_timeouts;

    uint8_t  acks_logged, infos_logged;
    uint16_t bad_pkts;
    uint16_t acks_sent;

    bool     bridge;                /* 转换器用法：能力交给上层，不做探测扫升压 */
    bool     caps_new;
    uint8_t  req_status;            /* 0 进行中，1 成功，2 失败 */
} s;
static bool g_bridge;               /* connect 期间（memset 之前）也要用 */

/* ---------------- 记录 ---------------- */

static void log_step(uint8_t code, uint8_t arg, uint16_t v, uint16_t i)
{
    /* 转换器用法里设备会频繁调压：请求过程由协议桥自己记录，这里只记连接与异常 */
    if (g_bridge && (code == UFS_REQUEST || code == UFS_ACCEPT || code == UFS_READY || code == UFS_SRC_INFO))
        return;
    ev_ufcs_step_t e = {code, arg, v, i, analog_vbus_mv()};
    evlog_add(EV_UFCS_STEP, &e, sizeof(e));
}

static void log_pkt(uint8_t dir, uint8_t baud, uint16_t ticks, const uint8_t *raw, uint8_t n)
{
    if (g_bridge && dir == 1)
        return;
    uint8_t buf[sizeof(ev_ufcs_pkt_t) + UFCS_MAX_PKT];
    ev_ufcs_pkt_t *h = (ev_ufcs_pkt_t *)buf;
    h->dir = dir;
    h->baud = baud;
    h->bit_ticks = ticks;
    h->len = n;
    memcpy(&buf[sizeof(*h)], raw, n);
    evlog_add(EV_UFCS_PKT, buf, sizeof(*h) + n);
}

/* ---------------- 发送事务 ---------------- */

static void tx_queue(const ufcs_msg_t *m)
{
    s.msg = *m;
    s.msg.num = s.num;
    s.msg.addr = s.tx_addr;
    s.rawn = ufcs_encode(&s.msg, s.raw);
    s.retry = 0;
    s.nck = false;
    s.txs = TX_QUEUED;
}

static void ctrl_queue(uint8_t cmd)
{
    ufcs_msg_t m;
    ufcs_make_ctrl(&m, s.tx_addr, s.num, cmd);
    tx_queue(&m);
}

static void end_session(uint8_t result);
static void tx_done(void);

static void tx_fail(void)
{
    s.txs = TX_NONE;
    if (s.flow == F_PING)
    {
        s.flow = F_DONE;        /* 由 ufcs_probe_boot 决定换参数重试 */
        return;
    }
    log_step(UFS_TIMEOUT, UFW_ACK, s.msg.cmd, 0);
    end_session(1);
}

/* 距最近一次收发结束的微秒数（收包时间与我方发送完成时间取较晚者） */
static uint32_t idle_us(uint32_t now)
{
    uint32_t a = s.last_act_us, b = ufcs_phy_tx_end_us();
    return now - ((int32_t)(a - b) > 0 ? a : b);
}

static void tx_step(void)
{
    uint32_t now = micros();

    /* 先发待回的 ACK/NCK */
    if (s.ack_pending && (int32_t)(now - s.ack_at_us) >= 0 && !ufcs_phy_tx_busy() && !ufcs_phy_rx_busy())
    {
        ufcs_msg_t a;
        uint8_t raw[8];
        ufcs_make_ctrl(&a, s.tx_addr, s.ack_num, s.ack_cmd);
        uint8_t n = ufcs_encode(&a, raw);
        if (ufcs_phy_send(raw, n))
        {
            if (s.ack_cmd != UFCS_C_ACK || s.acks_logged < LOG_ACKS)
            {
                s.acks_logged += s.ack_cmd == UFCS_C_ACK;
                log_pkt(1, s.tx_baud, 0, raw, n);
            }
            s.ack_pending = false;
            s.acks_sent++;
            /* 我方 ACK 也算一次发送，包间隔从其发完起算 */
        }
        return;
    }

    switch (s.txs)
    {
    case TX_QUEUED:
        if (s.ack_pending || ufcs_phy_tx_busy() || ufcs_phy_rx_busy())
            break;
        if (idle_us(now) < T_MSG_GAP_US)
            break;
        if (ufcs_phy_send(s.raw, s.rawn))
        {
            if (s.retry == 0)
                log_pkt(1, s.tx_baud, 0, s.raw, s.rawn);
            s.txs = TX_SENDING;
        }
        break;
    case TX_SENDING:
        if (!ufcs_phy_tx_busy())
        {
            s.last_act_us = ufcs_phy_tx_end_us();
            s.ack_deadline_us = s.last_act_us + T_ACK_RECEIVE_MS * 1000u;
            s.txs = TX_WAIT_ACK;
        }
        break;
    case TX_WAIT_ACK:
        if (s.nck || (int32_t)(now - s.ack_deadline_us) >= 0)
        {
            if (++s.retry > (s.flow == F_PING ? 20 : N_MSG_RETRY))
            {
                s.num = (s.num + 1) & 15;
                tx_fail();
            }
            else
            {
                s.nck = false;
                s.txs = TX_QUEUED;
            }
        }
        break;
    default:
        break;
    }
}

/* ---------------- 流程 ---------------- */

static void set_wait(wait_t w, uint32_t ms)
{
    s.wait = w;
    s.wait_deadline = millis() + ms;
}

static void start_ping(void)
{
    s.flow = F_PING;
    s.t_hs_ms = millis();
    ctrl_queue(UFCS_C_PING);
}

static void start_caps(void)
{
    s.flow = F_CAPS;
    ctrl_queue(UFCS_C_GET_OUTPUT_CAPS);
    set_wait(W_CAPS, 300);
}

/* 探测目标：先 5V，再各档最大电压（≤20V，1A 左右），最后回到 5V */
static void add_target(uint8_t mode_idx, uint16_t mv, uint16_t ma_pref)
{
    const ufcs_mode_t *md = &s.modes[mode_idx];
    if (s.ntargets >= MAX_TARGETS)
        return;
    if (!ufcs_mode_accepts(md, mv, ma_pref))
    {
        uint16_t lo = md->imin_10ma * 10u;
        lo += (md->ma_step - lo % md->ma_step) % md->ma_step;
        if (!ufcs_mode_accepts(md, mv, lo))
            return;
        ma_pref = lo;
    }
    s.targets[s.ntargets++] = (target_t){md->num, mv, ma_pref};
}

static void build_targets(void)
{
    s.ntargets = 0;
    int first = -1;
    for (uint8_t i = 0; i < s.nmodes; i++)
    {
        if (s.modes[i].vmin_10mv <= 500 && s.modes[i].vmax_10mv >= 500)
        {
            first = i;
            break;
        }
    }
    if (first < 0)
        first = 0;
    uint16_t v5 = 5000;
    if (s.modes[first].vmin_10mv > 500)
        v5 = s.modes[first].vmin_10mv * 10u;
    add_target(first, v5, 1000);
    for (uint8_t i = 0; i < s.nmodes; i++)
    {
        uint16_t v = s.modes[i].vmax_10mv * 10u;
        if (v > 20000)
            v = 20000;
        v -= v % s.modes[i].mv_step;
        if (v <= 5500 || v < s.modes[i].vmin_10mv * 10u)
            continue;
        add_target(i, v, 1000);
    }
    if (s.ntargets > 1)
        s.targets[s.ntargets++] = s.targets[0];
    s.ti = 0;
}

static void start_request(void)
{
    const target_t *t = &s.targets[s.ti];
    ufcs_msg_t m;
    ufcs_make_request(&m, 0, t->mode, t->mv, t->ma);
    tx_queue(&m);
    s.flow = F_REQ;
    s.t_req_ms = millis();
    set_wait(W_ACCEPT, 400);
    log_step(UFS_REQUEST, t->mode, t->mv, t->ma);
}

static void next_target(void)
{
    if (++s.ti < s.ntargets)
    {
        start_request();
        return;
    }
    s.flow = F_HOLD;
    s.last_poll_ms = millis();
    s.wait = W_NONE;
    log_step(UFS_END, 0, s.bad_pkts, 0);
    const ufcs_phy_stats_t *st = ufcs_phy_stats();
    log_step(UFS_PHY_STATS, (uint8_t)st->train_bad, st->train_ok, st->timeout + st->frame_err);
}

static void end_session(uint8_t result)
{
    s.wait = W_NONE;
    s.txs = TX_NONE;
    if (result == 1)
    {
        log_step(UFS_HARD_RESET, 1, 0, 0);
        ufcs_phy_hard_reset();
        const ufcs_phy_stats_t *st = ufcs_phy_stats();
        log_step(UFS_PHY_STATS, (uint8_t)st->train_bad, st->train_ok, st->timeout + st->frame_err);
    }
    log_step(UFS_END, result, s.bad_pkts, 0);
    s.flow = F_DONE;
}

static void tx_done(void)
{
    switch (s.flow)
    {
    case F_PING:
        log_step(UFS_PING_OK, s.tx_baud | (s.tx_addr << 4), (uint16_t)(millis() - s.t_hs_ms), 0);
        start_caps();
        break;
    case F_CAPS:
        set_wait(W_CAPS, T_RESPONSE_MS + 20);
        break;
    case F_REQ:
        set_wait(W_ACCEPT, T_RESPONSE_MS + 20);
        break;
    default:
        break;
    }
}

static void send_refuse(const ufcs_msg_t *refused, uint8_t reason)
{
    if (s.txs != TX_NONE)
        return;                         /* 探测固件不排队：忙时让充电器超时重发 */
    ufcs_msg_t r;
    ufcs_make_refuse(&r, s.tx_addr, s.num, refused, reason);
    tx_queue(&r);
}

static void on_message(const ufcs_msg_t *m)
{
    uint8_t rnum, rtype, rcmd, rreason;
    int16_t t1, t2;
    uint16_t mv, ma;

    if (m->type == UFCS_TYPE_CTRL)
    {
        switch (m->cmd)
        {
        case UFCS_C_ACCEPT:
            if (s.wait == W_ACCEPT)
            {
                s.t_accept_ms = millis();
                log_step(UFS_ACCEPT, 0, (uint16_t)(s.t_accept_ms - s.t_req_ms), 0);
                set_wait(W_READY, T_POWER_SUPPLY_MS + 30);
            }
            break;
        case UFCS_C_POWER_READY:
            if (s.wait == W_READY)
            {
                s.t_ready_ms = millis();
                s.wait = W_NONE;
                if (s.bridge)
                {
                    s.req_status = 1;           /* 电压是否到位由协议桥用 VBUS 采样验证 */
                    s.flow = F_HOLD;
                    s.last_poll_ms = millis();
                }
                else
                {
                    s.flow = F_SETTLE;
                    s.settle_at_ms = millis() + T_SETTLE_MS;
                }
            }
            break;
        case UFCS_C_PING:
            break;
        case UFCS_C_SOFT_RESET:
            /* 充电器软复位：消息编号清零，探测流程不重启 */
            s.num = 0;
            break;
        case UFCS_C_EXIT_UFCS:
            end_session(2);
            break;
        case UFCS_C_GET_SINK_INFO:
        {
            ufcs_msg_t r;
            ufcs_make_sink_info(&r, s.tx_addr, s.num, analog_vbus_mv(), 0);
            if (s.txs == TX_NONE)
                tx_queue(&r);
            break;
        }
        default:
            if (m->cmd >= UFCS_C_GET_CABLE_INFO)
                send_refuse(m, 0x02);
            break;
        }
        return;
    }
    if (m->type != UFCS_TYPE_DATA)
        return;

    switch (m->cmd)
    {
    case UFCS_D_OUTPUT_CAPS:
        if (s.wait == W_CAPS)
        {
            s.nmodes = ufcs_parse_caps(m, s.modes);
            log_step(UFS_CAPS, s.nmodes, 0, 0);
            s.wait = W_NONE;
            if (!s.nmodes)
            {
                end_session(1);
                break;
            }
            if (s.bridge)
            {
                s.caps_new = true;
                s.flow = F_HOLD;
                s.last_poll_ms = millis();
                break;
            }
            build_targets();
            if (!s.ntargets)
            {
                end_session(1);
                break;
            }
            start_request();
        }
        break;
    case UFCS_D_REFUSE:
        if (ufcs_parse_refuse(m, &rnum, &rtype, &rcmd, &rreason))
        {
            log_step(UFS_REFUSE, rreason, rcmd, rtype);
            if (s.flow == F_REQ && s.bridge)
            {
                s.wait = W_NONE;
                s.req_status = 2;
                s.flow = F_HOLD;
                s.last_poll_ms = millis();
            }
            else if (s.flow == F_REQ)
            {
                s.wait = W_NONE;
                next_target();
            }
            else if (s.flow == F_CAPS)
            {
                end_session(1);
            }
            else
            {
                s.wait = W_NONE;
            }
        }
        break;
    case UFCS_D_SOURCE_INFO:
        if (ufcs_parse_source_info(m, &t1, &t2, &mv, &ma))
        {
            if (s.wait == W_INFO)
                s.wait = W_NONE;
            s.info_timeouts = 0;
            if (s.infos_logged < LOG_INFOS)
            {
                s.infos_logged++;
                log_step(UFS_SRC_INFO, t1 == -999 ? 0x80 : (uint8_t)t1, mv, ma);
            }
        }
        break;
    case UFCS_D_POWER_CHANGE:
        break;                          /* 原始报文已记录 */
    case UFCS_D_VERIFY_REQUEST:
    case UFCS_D_DEVICE_INFO:
    case UFCS_D_ERROR_INFO:
    case UFCS_D_CABLE_INFO:
        send_refuse(m, 0x02);
        break;
    default:
        break;
    }
}

static void on_rx(const ufcs_rx_pkt_t *p)
{
    ufcs_msg_t m;
    bool crc_ok = false;
    bool ok = ufcs_decode(p->data, p->len, &m, &crc_ok);

    s.last_act_us = p->t_us;

    bool is_ack = ok && m.type == UFCS_TYPE_CTRL && (m.cmd == UFCS_C_ACK || m.cmd == UFCS_C_NCK);
    bool keep = !g_bridge || !ok || (m.type == UFCS_TYPE_DATA && m.cmd == UFCS_D_OUTPUT_CAPS);
    if (keep && (!is_ack || s.acks_logged < LOG_ACKS))
    {
        s.acks_logged += is_ack;
        log_pkt(0, p->baud, p->bit_ticks, p->data, p->len);
    }

    if (!ok)
    {
        s.bad_pkts++;
        if (!crc_ok && ufcs_frame_len(p->data, p->len) == p->len)
        {
            /* 长度对但 CRC 错：回 NCK（规范 6.7） */
            s.ack_pending = true;
            s.ack_cmd = UFCS_C_NCK;
            s.ack_num = (p->data[0] >> 1) & 15;
            s.ack_at_us = p->t_us + T_ACK_TX_US;
        }
        return;
    }
    if (m.type == UFCS_TYPE_CTRL && m.cmd == UFCS_C_ACK)
    {
        /* ACK 可能在我方还没观察到发送完成时就到了；这里立即结算，以免与随后紧跟的应答消息混淆 */
        if ((s.txs == TX_WAIT_ACK || s.txs == TX_SENDING) && m.num == s.msg.num)
        {
            s.txs = TX_NONE;
            s.num = (s.num + 1) & 15;
            tx_done();
        }
        return;
    }
    if (m.type == UFCS_TYPE_CTRL && m.cmd == UFCS_C_NCK)
    {
        if (s.txs == TX_WAIT_ACK && m.num == s.msg.num)
            s.nck = true;
        return;
    }
    s.ack_pending = true;
    s.ack_cmd = UFCS_C_ACK;
    s.ack_num = m.num;
    s.ack_at_us = p->t_us + T_ACK_TX_US;
    on_message(&m);
}

static void wait_timeout(void)
{
    uint8_t w = s.wait;
    s.wait = W_NONE;
    switch (w)
    {
    case W_CAPS:
        log_step(UFS_TIMEOUT, UFW_CAPS, 0, 0);
        end_session(1);
        break;
    case W_ACCEPT:
        log_step(UFS_TIMEOUT, UFW_ACCEPT, 0, 0);
        s.req_status = 2;
        end_session(1);
        break;
    case W_READY:
        log_step(UFS_TIMEOUT, UFW_READY, 0, 0);
        s.req_status = 2;
        end_session(1);
        break;
    case W_INFO:
        log_step(UFS_TIMEOUT, UFW_INFO, 0, 0);
        if (++s.info_timeouts >= 3)
            end_session(1);
        break;
    default:
        break;
    }
}

/* 前端 VBUS 监视：充电器断电时 VBUS 会在几 ms 内掉下来，趁板上还有余电马上把记录写进 Flash */
static void vbus_watch(void)
{
    static uint32_t last_ms;
    static bool was_ok;
    uint32_t now = millis();
    if (now == last_ms)
        return;
    last_ms = now;
    uint16_t v = analog_vbus_mv();
    if (v >= 4500)
    {
        was_ok = true;
    }
    else if (v < 3500 && was_ok)
    {
        was_ok = false;
        log_step(UFS_VBUS_LOST, s.flow, v, 0);
        evlog_panic_flush();
        s.flow = F_DONE;
    }
}

void ufcs_process(void)
{
    if (!s.up)
        return;

    vbus_watch();

    ufcs_phy_process();
    if (ufcs_phy_take_reset() && s.flow != F_DONE)
    {
        const ufcs_phy_stats_t *st = ufcs_phy_stats();
        log_step(UFS_HARD_RESET, 0, (uint16_t)(micros() - s.last_act_us) / 1000, s.acks_sent);
        log_step(UFS_PHY_STATS, (uint8_t)st->train_bad, st->train_ok, st->timeout + st->frame_err);
        end_session(2);
    }
    uint16_t rl = ufcs_phy_take_reset_len();
    if (rl)
        log_step(UFS_RESET_LEN, 0, rl, 0);

    ufcs_rx_pkt_t p;
    while (ufcs_phy_recv(&p))
        on_rx(&p);

    if (s.flow == F_DONE)
        return;

    tx_step();

    if (s.wait != W_NONE && (int32_t)(millis() - s.wait_deadline) > 0)
        wait_timeout();

    uint32_t now = millis();
    if (s.flow == F_SETTLE && (int32_t)(now - s.settle_at_ms) >= 0)
    {
        const target_t *t = &s.targets[s.ti];
        log_step(UFS_READY, t->mode, t->mv, (uint16_t)(s.t_ready_ms - s.t_accept_ms));
        next_target();
    }
    else if (s.flow == F_HOLD && s.txs == TX_NONE && s.wait == W_NONE && !s.ack_pending &&
             now - s.last_poll_ms >= T_POLL_MS)
    {
        s.last_poll_ms = now;
        ctrl_queue(UFCS_C_GET_SOURCE_INFO);
        set_wait(W_INFO, T_RESPONSE_MS + 60);
    }
}

bool ufcs_session_active(void)
{
    return s.up;
}

bool ufcs_flash_safe(void)
{
    if (!s.up)
        return true;
    if (s.flow == F_DONE)
        return true;
    /* Flash 写会让 CPU 停 5ms 左右：只在收发都空闲、离最近一次报文 30ms 以上、离下一步 100ms 以上时做，
     * 否则 ACK 会晚到，充电器会重发（实测 Source_Information 被重发过） */
    if (s.txs != TX_NONE || s.wait != W_NONE || s.ack_pending || ufcs_phy_rx_busy() || idle_us(micros()) < 30000)
        return false;
    if (s.flow == F_HOLD)
        return millis() - s.last_poll_ms < T_POLL_MS - 100;
    if (s.flow == F_SETTLE)
        return (int32_t)(s.settle_at_ms - millis()) > 40;
    return false;
}

/* ---------------- 启动流程 ---------------- */

bool ufcs_probe_boot(void)
{
    return ufcs_connect(false);
}

bool ufcs_connect(bool bridge)
{
    g_bridge = bridge;
    uint8_t flags = 0;
    if (!bridge)
    {
        bool dcp = ufcs_phy_dcp_detect(&flags);
        log_step(UFS_DCP, flags, 0, 0);
        (void)dcp;
    }
    uint8_t lv[4];
    ufcs_phy_scan(lv);
    log_step(UFS_SCAN_A, lv[0], lv[1], 0);
    log_step(UFS_SCAN_B, lv[2], lv[3], 0);
    /* 实测：UFCS 充电器在 D− 驱动 0.6V 时 D+ 读到约 0.67V（档位 13），没有 UFCS 的充电器读到 ≈0.05V；
     * dcp_detect 的下拉方式在这些充电器上误判为「不跟随」，只记录不作判据 */
    if (lv[2] < 8 && lv[3] < 8)
        return false;

    /* 每轮换一组（波特率，发送地址）：规范：115200 缺省，多次无回应才换档；地址字段含义待实测 */
    static const struct { uint8_t baud, addr; } plan[] = {
        {UFCS_BAUD_115200, UFCS_ADDR_SOURCE}, {UFCS_BAUD_57600, UFCS_ADDR_SOURCE},
        {UFCS_BAUD_38400, UFCS_ADDR_SOURCE},  {UFCS_BAUD_115200, UFCS_ADDR_SINK},
    };

    for (uint8_t r = 0; r < sizeof(plan) / sizeof(plan[0]); r++)
    {
        uint8_t tries = 0;
        if (!ufcs_phy_handshake(&tries))
        {
            log_step(UFS_HS_FAIL, tries, 0, 0);
            ufcs_phy_release();
            return false;                   /* 没有 D+ 拉高：不是 UFCS 充电器，交给 PD */
        }
        log_step(UFS_HS_OK, tries, 0, 0);

        memset(&s, 0, sizeof(s));
        s.up = true;
        s.bridge = bridge;
        s.tx_baud = plan[r].baud;
        s.tx_addr = plan[r].addr;
        ufcs_phy_set_baud(s.tx_baud);
        s.last_act_us = micros() - T_MSG_GAP_US;
        start_ping();

        uint32_t t0 = millis();
        while (s.flow == F_PING && millis() - t0 < 100)     /* tWaitPing 110~120ms 内充电器等 Ping */
            ufcs_process();
        if (s.flow != F_PING && s.flow != F_DONE)
            return true;

        log_step(UFS_PING_FAIL, s.tx_baud | (s.tx_addr << 4), 0, 0);
        ufcs_phy_hard_reset();
        log_step(UFS_HARD_RESET, 1, 0, 0);
        s.up = false;
        ufcs_phy_release();
        delay_ms(30);
    }
    return false;
}

/* ---------------- 转换器接口 ---------------- */

const ufcs_mode_t *ufcs_modes(uint8_t *n)
{
    *n = s.nmodes;
    return s.modes;
}

bool ufcs_take_caps(void)
{
    bool r = s.caps_new;
    s.caps_new = false;
    return r;
}

bool ufcs_idle(void)
{
    return s.up && s.flow == F_HOLD && s.txs == TX_NONE && s.wait == W_NONE;
}

bool ufcs_request(uint8_t mode, uint16_t mv, uint16_t ma)
{
    if (!ufcs_idle() || s.ack_pending)
        return false;
    s.targets[0] = (target_t){mode, mv, ma};
    s.ntargets = 1;
    s.ti = 0;
    s.req_status = 0;
    start_request();
    return true;
}

uint8_t ufcs_req_status(void)
{
    return s.req_status;
}

bool ufcs_dead(void)
{
    return s.up && s.flow == F_DONE;
}

void ufcs_close(void)
{
    s.up = false;
    ufcs_phy_release();
}

void ufcs_set_yield(void (*fn)(void))
{
    ufcs_phy_set_yield(fn);
}
