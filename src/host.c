#include "host.h"
#include "analog.h"
#include "be_source.h"
#include "board.h"
#include "bridge.h"
#include "cfg.h"
#include "evlog.h"
#include "fe_sink.h"
#include "flash_io.h"
#include "iap.h"
#include "pdinfo.h"
#include "power_sw.h"
#include "timebase.h"
#include "usb_hid.h"

enum
{
    CMD_INFO         = 0x01,
    CMD_STATUS       = 0x02,
    CMD_CFG_GET      = 0x10,
    CMD_CFG_SET      = 0x11,
    CMD_CFG_DEFAULT  = 0x12,
    CMD_CFGX_GET     = 0x13,    /* 协议 v11：扩展设置 cfg_ext_t（52 字节） */
    CMD_CFGX_SET     = 0x14,
    CMD_CFGF_GET     = 0x15,    /* 协议 v11：前端虚拟 E-Marker VDO（20 字节） */
    CMD_CFGF_SET     = 0x16,
    CMD_CFGP_GET     = 0x17,    /* 协议 v12：第二组自订 PDO（32 字节） */
    CMD_CFGP_SET     = 0x18,
    CMD_LOG_SESSIONS = 0x20,
    CMD_LOG_READ     = 0x21,
    CMD_LOG_CLEAR    = 0x22,
    /* CMD_SYS_INFO、CMD_ENTER_BL 见 iap.h */
};

enum
{
    ST_OK      = 0,
    ST_BAD_CMD = 1,
    ST_BAD_ARG = 2,
};

#define RSP_PAYLOAD     (USB_HID_REPORT_LEN - 3)
#define LOG_CHUNK       56

typedef struct
{
    uint8_t *p;
    uint8_t n;
} wr_t;

static void put8(wr_t *w, uint8_t v)
{
    if (w->n < RSP_PAYLOAD)
        w->p[w->n++] = v;
}

static void put16(wr_t *w, uint16_t v)
{
    put8(w, v & 0xFF);
    put8(w, v >> 8);
}

static void put32(wr_t *w, uint32_t v)
{
    put16(w, v & 0xFFFF);
    put16(w, v >> 16);
}

static void put_bytes(wr_t *w, const void *src, uint8_t len)
{
    const uint8_t *s = src;
    while (len--)
        put8(w, *s++);
}

static uint8_t cmd_status(const uint8_t *arg, wr_t *w)
{
    if (arg[0] == 4)
    {
        /* 协议 v10：后端线材 E-Marker：status u8, flags u8（bit0 硬件可供 VCONN，bit1 声明 5A，bit2 已按 5A 放宽后端电流上限，bit3 另一根 CC 脚有 Ra）, n u8, vdo u32[n] */
        uint32_t vdo[BE_CABLE_VDOS];
        uint8_t n = be_cable_vdos(vdo);
        put8(w, be_cable_status());
        put8(w, (be_vconn_available() ? 0x01 : 0) | (be_cable_5a() ? 0x02 : 0) | (bridge_back_max_ma() > cfg()->max_ma ? 0x04 : 0) | (be_cable_ra() ? 0x08 : 0));
        put8(w, n);
        for (uint8_t i = 0; i < n; i++)
            put32(w, vdo[i]);
        return ST_OK;
    }
    if (arg[0] == 3)
    {
        /* PD 信息透传：设备电量、双方身份（格式见 pdinfo_status） */
        uint8_t buf[RSP_PAYLOAD];
        put_bytes(w, buf, pdinfo_status(buf));
        return ST_OK;
    }
    if (arg[0] == 2)
    {
        /* 协议 v4：前端 EPR 能力表 8 号起的 PDO */
        uint8_t n;
        const pdo_t *caps = fe_caps(&n);
        uint8_t k = n > PD_MAX_DATA_OBJS ? n - PD_MAX_DATA_OBJS : 0;
        put8(w, k);
        for (uint8_t i = 0; i < k; i++)
            put32(w, caps[PD_MAX_DATA_OBJS + i].raw);
        return ST_OK;
    }
    if (arg[0] == 1)
    {
        uint8_t blob[PD_MAX_DATA_OBJS * 5];
        uint8_t n = bridge_back_caps_peek(blob);
        put8(w, n);
        put_bytes(w, blob, n * 5);
        return ST_OK;
    }

    uint32_t be_rdo;
    uint16_t be_mv;
    bool back_contract = bridge_back_contract(&be_rdo, &be_mv);
    uint32_t fe_rdo = fe_contract_rdo();
    uint8_t flags = (fe_is_legacy() ? 0x01 : 0) | (be_attached() ? 0x02 : 0) | (power_sw_is_on() ? 0x04 : 0) |
                    (back_contract ? 0x08 : 0) | (fe_rdo ? 0x10 : 0) | (bridge_rear_avs_2nd_state() << 5) | (fe_epr_mode() ? 0x80 : 0);

    put8(w, fe_state_code());
    put8(w, be_state_code());
    put8(w, flags);
    put16(w, analog_vbus_mv());
    put16(w, analog_current_ma());
    put32(w, fe_rdo);
    put32(w, be_rdo);
    put16(w, be_mv);

    uint8_t n;
    const pdo_t *caps = fe_caps(&n);
    if (n > PD_MAX_DATA_OBJS)
        n = PD_MAX_DATA_OBJS;   /* 其余见 arg 2 */
    put8(w, n);
    for (uint8_t i = 0; i < n; i++)
        put32(w, caps[i].raw);
    put16(w, analog_fe_cc_mv());    /* 协议 v3：前端 CC 电压（Rp，SinkTxOK/NG） */
    return ST_OK;
}

static uint8_t cmd_cfg_set(const cfg_t *c)
{
    if (!cfg_set(c))
        return ST_BAD_ARG;
    bridge_on_cfg_changed();
    return ST_OK;
}

static uint8_t cmd_cfgx_set(const uint8_t *arg, bool front)
{
    cfg_ext_t x = *cfg_ext();
    if (front)
        memcpy(x.fcable, arg, CFGF_SIZE);
    else
        memcpy(&x, arg, CFGX_BASE_SIZE);
    if (!cfg_ext_set(&x))
        return ST_BAD_ARG;
    bridge_on_cfg_changed();
    return ST_OK;
}

static bool enter_bl_pending;

static uint8_t handle(const uint8_t *req, wr_t *w)
{
    const uint8_t *arg = &req[2];
    switch (req[0])
    {
    case CMD_INFO:
        put8(w, HOST_PROTO_VERSION);
        put16(w, FW_VERSION);
        put16(w, evlog_session());
        put32(w, millis());
        put16(w, evlog_dropped());
        put8(w, (cfg_save_pending() ? 0x01 : 0) | (evlog_clearing() ? 0x02 : 0));
        put8(w, evlog_used_pages());
        return ST_OK;

    case CMD_STATUS:
        return cmd_status(arg, w);

    case CMD_CFG_GET:
        put_bytes(w, cfg(), sizeof(cfg_t));
        put8(w, cfg_save_pending());
        return ST_OK;

    case CMD_CFG_SET:
    {
        cfg_t c;
        memcpy(&c, arg, sizeof(c));
        return cmd_cfg_set(&c);
    }

    case CMD_CFG_DEFAULT:
    {
        cfg_t c;
        cfg_ext_t x;
        cfg_defaults(&c);
        cfg_ext_defaults(&x);
        cfg_pdo2_t p2 = {0};
        cfg_pdo2_set(&p2);
        return cfg_ext_set(&x) ? cmd_cfg_set(&c) : ST_BAD_ARG;
    }

    case CMD_CFGX_GET:
        put_bytes(w, cfg_ext(), CFGX_BASE_SIZE);
        put8(w, cfg_save_pending());
        return ST_OK;

    case CMD_CFGF_GET:
        put_bytes(w, cfg_ext()->fcable, CFGF_SIZE);
        put8(w, cfg_save_pending());
        return ST_OK;

    case CMD_CFGP_GET:
        put_bytes(w, cfg_pdo2(), CFGP_SIZE);
        put8(w, cfg_save_pending());
        return ST_OK;

    case CMD_CFGP_SET:
    {
        cfg_pdo2_t p;
        memcpy(&p, arg, CFGP_SIZE);
        if (!cfg_pdo2_set(&p))
            return ST_BAD_ARG;
        bridge_on_cfg_changed();
        return ST_OK;
    }

    case CMD_CFGX_SET:
        return cmd_cfgx_set(arg, false);

    case CMD_CFGF_SET:
        return cmd_cfgx_set(arg, true);

    case CMD_LOG_SESSIONS:
    {
        evlog_session_t s[9];
        uint8_t n = evlog_sessions(s, sizeof(s) / sizeof(s[0]));
        uint8_t ram = (evlog_page(EVLOG_RAM_CURRENT) ? 0x01 : 0) | (evlog_page(EVLOG_RAM_SEALED0) ? 0x02 : 0) |
                      (evlog_page(EVLOG_RAM_SEALED1) ? 0x04 : 0);
        put16(w, evlog_session());
        put8(w, ram);
        put8(w, n);
        for (uint8_t i = 0; i < n; i++)
        {
            put16(w, s[i].session);
            put8(w, s[i].first_idx);
            put8(w, s[i].pages);
        }
        return ST_OK;
    }

    case CMD_LOG_READ:
    {
        uint8_t idx = arg[0], off = arg[1];
        const uint8_t *page = evlog_page(idx);
        if (!page || off >= FLASH_PAGE_SIZE)
            return ST_BAD_ARG;
        uint8_t len = FLASH_PAGE_SIZE - off;
        if (len > LOG_CHUNK)
            len = LOG_CHUNK;
        put8(w, idx);
        put8(w, off);
        put8(w, len);
        put_bytes(w, page + off, len);
        return ST_OK;
    }

    case CMD_LOG_CLEAR:
        evlog_clear();
        return ST_OK;

    case CMD_SYS_INFO:
    {
        const iap_hdr_t *h = iap_hdr();
        bool ok = iap_hdr_ok(h);
        put8(w, 1);                     /* 1 = APP */
        put16(w, 0);                    /* BL 版本只有 BL 模式下报告 */
        put8(w, IAP_PROTO);
        put8(w, 1);
        put16(w, FW_VERSION);
        put32(w, ok ? h->size : 0);
        put32(w, APP_BASE);
        put32(w, APP_MAX_SIZE);
        return ST_OK;
    }

    case CMD_ENTER_BL:
        if (arg[0] != 'B' || arg[1] != 'L')
            return ST_BAD_ARG;
        enter_bl_pending = true;        /* 应答发出后再写标志并复位 */
        return ST_OK;

    default:
        return ST_BAD_CMD;
    }
}

void host_process(void)
{
    static uint8_t rsp[USB_HID_REPORT_LEN];
    static bool rsp_pending;
    uint8_t req[USB_HID_REPORT_LEN];

    if (rsp_pending)
    {
        if (!usb_hid_send(rsp))
            return;
        rsp_pending = false;
    }
    if (!usb_hid_receive(req))
        return;

    memset(rsp, 0, sizeof(rsp));
    wr_t w = {&rsp[3], 0};
    rsp[0] = req[0] | 0x80;
    rsp[1] = req[1];
    rsp[2] = handle(req, &w);
    rsp_pending = !usb_hid_send(rsp);
    if (enter_bl_pending)
    {
        /* 等应答被主机取走（最多 100ms），再写标志页并复位；后端输出随复位断开 */
        uint32_t t0 = millis();
        while (rsp_pending || usb_hid_tx_busy())
        {
            if (rsp_pending && usb_hid_send(rsp))
                rsp_pending = false;
            if (millis() - t0 > 100)
                break;
        }
        delay_ms(20);           /* 让主机把应答读走再复位 */
        iap_enter_bootloader();
    }
}
