#include "usb_hid.h"
#include "board.h"
#include "ch32m030_usb.h"
#include "timebase.h"

#define EP0_SIZE        64
#define USB_VID         0x1209      /* pid.codes */
#define USB_PID         0x0001      /* pid.codes 测试 ID，正式发布前申请专属 PID */
#define USB_BCD         0x0200

#define ESIG_UNIID      0x1FFFF3A8u /* 参考手册 §18.2：96 位 UID，3 个字 */

typedef struct __attribute__((packed))
{
    uint8_t  bmRequestType;
    uint8_t  bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} setup_t;

static const uint8_t dev_desc[18] = {
    18, USB_DESCR_TYP_DEVICE, 0x00, 0x02, 0x00, 0x00, 0x00, EP0_SIZE,
    USB_VID & 0xFF, USB_VID >> 8, USB_PID & 0xFF, USB_PID >> 8, USB_BCD & 0xFF, USB_BCD >> 8,
    1, 2, 3, 1,
};

/* 厂商自定义 Usage Page 0xFF00：64 字节输入报告 + 64 字节输出报告，无 Report ID */
static const uint8_t report_desc[] = {
    0x06, 0x00, 0xFF,           /* Usage Page (0xFF00) */
    0x09, 0x01,                 /* Usage (0x01) */
    0xA1, 0x01,                 /* Collection (Application) */
    0x09, 0x02,                 /*   Usage (0x02) */
    0x15, 0x00,                 /*   Logical Minimum (0) */
    0x26, 0xFF, 0x00,           /*   Logical Maximum (255) */
    0x75, 0x08,                 /*   Report Size (8) */
    0x95, USB_HID_REPORT_LEN,   /*   Report Count (64) */
    0x81, 0x02,                 /*   Input (Data, Var, Abs) */
    0x09, 0x03,                 /*   Usage (0x03) */
    0x15, 0x00,
    0x26, 0xFF, 0x00,
    0x75, 0x08,
    0x95, USB_HID_REPORT_LEN,
    0x91, 0x02,                 /*   Output (Data, Var, Abs) */
    0xC0,                       /* End Collection */
};

#define CFG_DESC_LEN    (9 + 9 + 9 + 7 + 7)
#define HID_DESC_OFFSET 18

static const uint8_t cfg_desc[CFG_DESC_LEN] = {
    9, USB_DESCR_TYP_CONFIG, CFG_DESC_LEN, 0, 1, 1, 0, 0x80, 50,           /* 总线供电 100mA */
    9, USB_DESCR_TYP_INTERF, 0, 0, 2, 0x03, 0x00, 0x00, 0,                  /* HID，无子类 */
    9, USB_DESCR_TYP_HID, 0x11, 0x01, 0x00, 1, USB_DESCR_TYP_REPORT,
    sizeof(report_desc) & 0xFF, sizeof(report_desc) >> 8,
    7, USB_DESCR_TYP_ENDP, 0x81, 0x03, USB_HID_REPORT_LEN, 0, 1,            /* EP1 IN，中断，1ms */
    7, USB_DESCR_TYP_ENDP, 0x01, 0x03, USB_HID_REPORT_LEN, 0, 1,            /* EP1 OUT */
};

#ifdef BOOTLOADER
#define USB_PRODUCT_NAME "PPStoAVS Bootloader"
#else
#define USB_PRODUCT_NAME "PPStoAVS Bridge"
#endif
static const char *const str_ascii[] = {NULL, "PPStoAVS", USB_PRODUCT_NAME, NULL};
static const uint8_t zero_report[USB_HID_REPORT_LEN];

static uint8_t ep0_buf[EP0_SIZE] __attribute__((aligned(4)));
static uint8_t ep1_buf[2 * USB_HID_REPORT_LEN] __attribute__((aligned(4)));   /* [0,64) OUT，[64,128) IN */

static uint8_t str_buf[2 + 2 * 32];
static char serial[25];
static uint8_t ctl_buf[2];

static setup_t req;
static const uint8_t *tx_ptr;
static uint16_t tx_left;
static uint8_t dev_addr;
static volatile uint8_t dev_config;
static uint8_t hid_idle;

static bool on_rear;
static bool ever_config;        /* 当前接口上曾被主机配置过 */
static bool route_done;         /* 前端超时已处理（切了或没有 SEL） */
static int8_t has_sel = -1;
static uint32_t init_ms;
static volatile bool in_busy;
static volatile bool rx_ready;
static uint8_t rx_buf[USB_HID_REPORT_LEN];

void USBFS_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

static void make_string(const char *s)
{
    uint8_t n = 0;
    while (s[n] && n < 32)
    {
        str_buf[2 + 2 * n] = (uint8_t)s[n];
        str_buf[3 + 2 * n] = 0;
        n++;
    }
    str_buf[0] = 2 + 2 * n;
    str_buf[1] = USB_DESCR_TYP_STRING;
}

static void ep1_reset(void)
{
    USBFSD->UEP1_TX_CTRL = USBFS_UEP_T_RES_NAK;
    USBFSD->UEP1_RX_CTRL = rx_ready ? USBFS_UEP_R_RES_NAK : USBFS_UEP_R_RES_ACK;
    USBFSD->UEP1_TX_LEN = 0;
    in_busy = false;
}

static void endp_init(void)
{
    USBFSD->UEP4_1_MOD = USBFS_UEP1_RX_EN | USBFS_UEP1_TX_EN;
    USBFSD->UEP0_DMA = (uint32_t)ep0_buf;
    USBFSD->UEP1_DMA = (uint32_t)ep1_buf;
    USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_RES_NAK;
    USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_RES_ACK;
    ep1_reset();
}

void usb_hid_init(void)
{
    static const char hex[] = "0123456789ABCDEF";
    const uint32_t *uid = (const uint32_t *)ESIG_UNIID;
    for (uint8_t i = 0; i < 24; i++)
        serial[i] = hex[(uid[i / 8] >> (28 - (i % 8) * 4)) & 0xF];
    serial[24] = 0;

    (void)usb_hid_has_sel();    /* 第一次初始化时检测（之后 SEL 由 usb_hid_route 控制，不再碰） */
    init_ms = millis();
    ever_config = false;
    RCC_HBPeriphClockCmd(RCC_HBPeriph_USBFS, ENABLE);
    USBFSD->BASE_CTRL = USBFS_UC_RESET_SIE | USBFS_UC_CLR_ALL;
    delay_us(10);
    USBFSD->BASE_CTRL = 0;
    USBFSD->INT_EN = USBFS_UIE_SUSPEND | USBFS_UIE_BUS_RST | USBFS_UIE_TRANSFER;
    USBFSD->BASE_CTRL = USBFS_UC_DEV_PU_EN | USBFS_UC_INT_BUSY | USBFS_UC_DMA_EN;
    endp_init();
    USBFSD->UDEV_CTRL = USBFS_UD_PD_DIS | USBFS_UD_PORT_EN;

    NVIC_SetPriority(USBFS_IRQn, 0x80);     /* 低于 USBPD（0x00），PD 的 GoodCRC 优先 */
    NVIC_EnableIRQ(USBFS_IRQn);
}

void usb_hid_route(bool rear)
{
    (void)usb_hid_has_sel();    /* 检测要在驱动 SEL 之前 */
    NVIC_DisableIRQ(USBFS_IRQn);
    USBFSD->BASE_CTRL &= ~USBFS_UC_DEV_PU_EN;  /* 先断开上拉，切换后由 usb_hid_init 重新接上 */
    USBFSD->UDEV_CTRL = 0;

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOA, ENABLE);
    if (rear)
        GPIO_ResetBits(USB_SEL_PORT, USB_SEL_PIN);
    else
        GPIO_SetBits(USB_SEL_PORT, USB_SEL_PIN);
    GPIO_InitTypeDef g = {0};
    g.GPIO_Pin = USB_SEL_PIN;
    g.GPIO_Speed = GPIO_Speed_30MHz;
    g.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(USB_SEL_PORT, &g);
    delay_us(200);

    on_rear = rear;
    dev_config = 0;
    dev_addr = 0;
    rx_ready = false;
    in_busy = false;
    tx_left = 0;
    usb_hid_init();
}

bool usb_hid_on_rear(void)
{
    return on_rear;
}

bool usb_hid_has_sel(void)
{
    if (has_sel < 0)
    {
        /* 先输出低电平把引脚放电（PA12 没有内部下拉），再改浮空：新板 R31 100K 上拉（约 1µs 时间常数）20µs 内拉高；
         * 旧板引脚悬空，保持低。检测在 USB 初始化之前，SEL 短暂为 0 没有影响 */
        RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOA, ENABLE);
        GPIO_ResetBits(USB_SEL_PORT, USB_SEL_PIN);
        GPIO_InitTypeDef g = {0};
        g.GPIO_Pin = USB_SEL_PIN;
        g.GPIO_Speed = GPIO_Speed_30MHz;
        g.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_Init(USB_SEL_PORT, &g);
        delay_us(50);
        g.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_Init(USB_SEL_PORT, &g);
        delay_us(20);
        has_sel = GPIO_ReadInputDataBit(USB_SEL_PORT, USB_SEL_PIN) ? 1 : 0;
    }
    return has_sel > 0;
}

uint8_t usb_hid_poll_route(void)
{
    if (dev_config && !ever_config)
    {
        ever_config = true;
        if (on_rear)
            return USB_ROUTE_REAR_CFG;
    }
    if (on_rear || ever_config || route_done || millis() - init_ms < USB_FRONT_TIMEOUT_MS)
        return USB_ROUTE_NONE;
    route_done = true;
    if (!usb_hid_has_sel())
        return USB_ROUTE_NO_SEL;
    usb_hid_route(true);
    return USB_ROUTE_REAR;
}

bool usb_hid_configured(void)
{
    return dev_config != 0;
}

bool usb_hid_receive(uint8_t *buf)
{
    if (!rx_ready)
        return false;
    memcpy(buf, rx_buf, USB_HID_REPORT_LEN);
    NVIC_DisableIRQ(USBFS_IRQn);
    rx_ready = false;
    USBFSD->UEP1_RX_CTRL = (USBFSD->UEP1_RX_CTRL & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_ACK;
    NVIC_EnableIRQ(USBFS_IRQn);
    return true;
}

bool usb_hid_tx_busy(void)
{
    return in_busy;
}

bool usb_hid_send(const uint8_t *buf)
{
    if (!dev_config || in_busy)
        return false;
    memcpy(&ep1_buf[USB_HID_REPORT_LEN], buf, USB_HID_REPORT_LEN);
    NVIC_DisableIRQ(USBFS_IRQn);
    in_busy = true;
    USBFSD->UEP1_TX_LEN = USB_HID_REPORT_LEN;
    USBFSD->UEP1_TX_CTRL = (USBFSD->UEP1_TX_CTRL & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_ACK;
    NVIC_EnableIRQ(USBFS_IRQn);
    return true;
}

static void ep0_send_chunk(bool first)
{
    uint16_t n = tx_left > EP0_SIZE ? EP0_SIZE : tx_left;
    memcpy(ep0_buf, tx_ptr, n);
    tx_ptr += n;
    tx_left -= n;
    USBFSD->UEP0_TX_LEN = n;
    if (first)
        USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_TOG | USBFS_UEP_T_RES_ACK;
    else
        USBFSD->UEP0_TX_CTRL ^= USBFS_UEP_T_TOG;
}

/* 返回 false 表示不支持（STALL） */
static bool get_descriptor(const uint8_t **p, uint16_t *len)
{
    uint8_t type = req.wValue >> 8;
    uint8_t index = req.wValue & 0xFF;
    switch (type)
    {
    case USB_DESCR_TYP_DEVICE:
        *p = dev_desc;
        *len = sizeof(dev_desc);
        return true;
    case USB_DESCR_TYP_CONFIG:
        *p = cfg_desc;
        *len = sizeof(cfg_desc);
        return true;
    case USB_DESCR_TYP_HID:
        *p = &cfg_desc[HID_DESC_OFFSET];
        *len = 9;
        return true;
    case USB_DESCR_TYP_REPORT:
        *p = report_desc;
        *len = sizeof(report_desc);
        return true;
    case USB_DESCR_TYP_STRING:
        if (index == 0)
        {
            str_buf[0] = 4;
            str_buf[1] = USB_DESCR_TYP_STRING;
            str_buf[2] = 0x09;  /* 0x0409 英语（美国） */
            str_buf[3] = 0x04;
        }
        else if (index <= 3)
        {
            make_string(index == 3 ? serial : str_ascii[index]);
        }
        else
        {
            return false;
        }
        *p = str_buf;
        *len = str_buf[0];
        return true;
    default:
        return false;
    }
}

static void ep0_setup(void)
{
    memcpy(&req, ep0_buf, sizeof(req));
    USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_TOG | USBFS_UEP_T_RES_NAK;
    USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_TOG | USBFS_UEP_R_RES_NAK;

    const uint8_t *data = NULL;
    uint16_t len = 0;
    bool ok = true;
    uint8_t type = req.bmRequestType & USB_REQ_TYP_MASK;
    uint8_t recip = req.bmRequestType & USB_REQ_RECIP_MASK;

    if (type == USB_REQ_TYP_CLASS)
    {
        switch (req.bRequest)
        {
        case HID_GET_REPORT:
            data = zero_report;     /* 数据只走中断端点；控制端点轮询返回全 0 */
            len = USB_HID_REPORT_LEN;
            break;
        case HID_SET_REPORT:
        case HID_SET_PROTOCOL:
            break;
        case HID_SET_IDLE:
            hid_idle = req.wValue >> 8;
            break;
        case HID_GET_IDLE:
            ctl_buf[0] = hid_idle;
            data = ctl_buf;
            len = 1;
            break;
        case HID_GET_PROTOCOL:
            ctl_buf[0] = 1;     /* Report protocol */
            data = ctl_buf;
            len = 1;
            break;
        default:
            ok = false;
            break;
        }
    }
    else if (type == USB_REQ_TYP_STANDARD)
    {
        switch (req.bRequest)
        {
        case USB_GET_DESCRIPTOR:
            ok = get_descriptor(&data, &len);
            break;
        case USB_SET_ADDRESS:
            dev_addr = req.wValue & 0x7F;
            break;
        case USB_GET_CONFIGURATION:
            ctl_buf[0] = dev_config;
            data = ctl_buf;
            len = 1;
            break;
        case USB_SET_CONFIGURATION:
            dev_config = req.wValue & 0xFF;
            ep1_reset();
            break;
        case USB_GET_INTERFACE:
            ctl_buf[0] = 0;
            data = ctl_buf;
            len = 1;
            break;
        case USB_SET_INTERFACE:
            break;
        case USB_GET_STATUS:
            ctl_buf[0] = 0;
            ctl_buf[1] = 0;
            if (recip == USB_REQ_RECIP_ENDP)
            {
                uint8_t ep = req.wIndex & 0xFF;
                if ((ep == 0x81 && (USBFSD->UEP1_TX_CTRL & USBFS_UEP_T_RES_MASK) == USBFS_UEP_T_RES_STALL) ||
                    (ep == 0x01 && (USBFSD->UEP1_RX_CTRL & USBFS_UEP_R_RES_MASK) == USBFS_UEP_R_RES_STALL))
                    ctl_buf[0] = 1;
            }
            data = ctl_buf;
            len = 2;
            break;
        case USB_CLEAR_FEATURE:
        case USB_SET_FEATURE:
            if (recip == USB_REQ_RECIP_ENDP && req.wValue == USB_REQ_FEAT_ENDP_HALT)
            {
                uint8_t ep = req.wIndex & 0xFF;
                bool set = req.bRequest == USB_SET_FEATURE;
                if (ep == 0x81)
                {
                    USBFSD->UEP1_TX_CTRL = set ? USBFS_UEP_T_RES_STALL : USBFS_UEP_T_RES_NAK;
                    in_busy = false;
                }
                else if (ep == 0x01)
                {
                    USBFSD->UEP1_RX_CTRL = set ? USBFS_UEP_R_RES_STALL
                                               : (rx_ready ? USBFS_UEP_R_RES_NAK : USBFS_UEP_R_RES_ACK);
                }
                else if ((ep & 0x7F) != 0)
                {
                    ok = false;
                }
            }
            else if (!(recip == USB_REQ_RECIP_DEVICE && req.bRequest == USB_CLEAR_FEATURE))
            {
                ok = false;     /* 不支持远程唤醒 */
            }
            break;
        default:
            ok = false;
            break;
        }
    }
    else
    {
        ok = false;
    }

    if (!ok)
    {
        USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_TOG | USBFS_UEP_T_RES_STALL;
        USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_TOG | USBFS_UEP_R_RES_STALL;
        return;
    }

    if (req.bmRequestType & USB_REQ_TYP_IN)
    {
        tx_ptr = data;
        tx_left = req.wLength < len ? req.wLength : len;
        ep0_send_chunk(true);
    }
    else if (req.wLength == 0)
    {
        USBFSD->UEP0_TX_LEN = 0;    /* 状态阶段：IN 零长度包 */
        USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_TOG | USBFS_UEP_T_RES_ACK;
    }
    else
    {
        USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_TOG | USBFS_UEP_R_RES_ACK;   /* 等待 OUT 数据（SET_REPORT） */
    }
}

static void ep0_in(void)
{
    if (req.bmRequestType & USB_REQ_TYP_IN)
    {
        if (tx_left == 0)
            USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_TOG | USBFS_UEP_R_RES_ACK;  /* 准备状态阶段 OUT */
        ep0_send_chunk(false);  /* 剩余数据；刚好整包结束时补零长度包 */
    }
    else
    {
        /* 无数据阶段请求的状态阶段完成 */
        if ((req.bmRequestType & USB_REQ_TYP_MASK) == USB_REQ_TYP_STANDARD && req.bRequest == USB_SET_ADDRESS)
            USBFSD->DEV_ADDR = (USBFSD->DEV_ADDR & USBFS_UDA_GP_BIT) | dev_addr;
        USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_RES_NAK;
    }
}

static void ep0_out(void)
{
    if (req.bmRequestType & USB_REQ_TYP_IN)
        return;     /* IN 传输的状态阶段 */

    /* SET_REPORT 数据阶段：与中断 OUT 端点同样处理 */
    if (!rx_ready)
    {
        uint16_t n = USBFSD->RX_LEN;
        memset(rx_buf, 0, sizeof(rx_buf));
        memcpy(rx_buf, ep0_buf, n > USB_HID_REPORT_LEN ? USB_HID_REPORT_LEN : n);
        rx_ready = true;
        USBFSD->UEP1_RX_CTRL = (USBFSD->UEP1_RX_CTRL & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_NAK;
    }
    USBFSD->UEP0_TX_LEN = 0;
    USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_TOG | USBFS_UEP_T_RES_ACK;
}

static void ep1_out(void)
{
    uint16_t n = USBFSD->RX_LEN;
    USBFSD->UEP1_RX_CTRL ^= USBFS_UEP_R_TOG;
    if (!rx_ready)
    {
        memset(rx_buf, 0, sizeof(rx_buf));
        memcpy(rx_buf, ep1_buf, n > USB_HID_REPORT_LEN ? USB_HID_REPORT_LEN : n);
        rx_ready = true;
    }
    /* 主循环取走之前 NAK，防止覆盖 */
    USBFSD->UEP1_RX_CTRL = (USBFSD->UEP1_RX_CTRL & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_NAK;
}

static void ep1_in(void)
{
    USBFSD->UEP1_TX_CTRL = (USBFSD->UEP1_TX_CTRL & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_NAK;
    USBFSD->UEP1_TX_CTRL ^= USBFS_UEP_T_TOG;
    in_busy = false;
}

void USBFS_IRQHandler(void)
{
    uint8_t fg = USBFSD->INT_FG;
    uint8_t st = USBFSD->INT_ST;

    if (fg & USBFS_UIF_TRANSFER)
    {
        uint8_t ep = st & USBFS_UIS_ENDP_MASK;
        switch (st & USBFS_UIS_TOKEN_MASK)
        {
        case USBFS_UIS_TOKEN_SETUP:
            ep0_setup();
            break;
        case USBFS_UIS_TOKEN_IN:
            if (ep == 0)
                ep0_in();
            else if (ep == 1)
                ep1_in();
            break;
        case USBFS_UIS_TOKEN_OUT:
            if (st & USBFS_UIS_TOG_OK)
            {
                if (ep == 0)
                    ep0_out();
                else if (ep == 1)
                    ep1_out();
            }
            break;
        default:
            break;
        }
        USBFSD->INT_FG = USBFS_UIF_TRANSFER;
    }
    else if (fg & USBFS_UIF_BUS_RST)
    {
        dev_config = 0;
        dev_addr = 0;
        USBFSD->DEV_ADDR = 0;
        endp_init();
        USBFSD->INT_FG = USBFS_UIF_BUS_RST;
    }
    else if (fg & USBFS_UIF_SUSPEND)
    {
        USBFSD->INT_FG = USBFS_UIF_SUSPEND;
    }
    else
    {
        USBFSD->INT_FG = fg;
    }
}
