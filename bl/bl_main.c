/*
 * PPStoAVS Bootloader（在线升级，协议见 src/iap.h）
 *
 * 复位后先运行：有效程序且没有进入 BL 的标志 → 直接跳转；否则留在 BL 模式，用与 APP 相同的 USB HID
 * （D+/D-，VID:PID 1209:0001，产品名 "PPStoAVS Bootloader"）等上位机刷写。
 * 新板 USB 切换：APP 进 BL 时 USB 在后端（标志页第 2 字 'REAR'）→ 直接用后端；否则同 APP，前端 3s 没被配置再切到后端。
 * 功率开关始终关断（后端无输出），PD 不工作。
 */
#include "board.h"
#include "iap.h"
#include "timebase.h"
#include "usb_hid.h"

#define ST_OK       IAP_ST_OK
#define RSP_PAYLOAD (USB_HID_REPORT_LEN - 3)

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

static uint32_t get32(const uint8_t *p)
{
    return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* 尽早把 MOS 栅极拉低（与 APP 的 power_sw_early_off 相同） */
static void gate_off(void)
{
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOB, ENABLE);
    GPIO_SetBits(GATE_GPIO_PORT, GATE_GPIO_PIN);
    GPIO_InitTypeDef gpio = {0};
    gpio.GPIO_Pin = GATE_GPIO_PIN;
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GATE_GPIO_PORT, &gpio);
    GPIO_SetBits(GATE_GPIO_PORT, GATE_GPIO_PIN);
}

/* ---------------- 接收状态 ---------------- */
static bool active;                                 /* 已 BEGIN，正在接收 */
static uint32_t app_size, app_crc, recv;
static uint16_t app_ver;
static uint32_t page_buf[FLASH_PAGE_SIZE / 4];
static bool boot_pending;
static bool start_rear;                             /* 在 SystemInit 里设置（.bss 已在之前清零） */

/* 把缓冲里 fill 个字节写成一页（不足补 0xFF）。page_no 从 0 开始 */
static bool flush_page(uint32_t page_no, uint32_t fill)
{
    uint8_t *b = (uint8_t *)page_buf;
    memset(b + fill, 0xFF, FLASH_PAGE_SIZE - fill);
    uint32_t addr = APP_ADDR + page_no * FLASH_PAGE_SIZE;
    return flash_page_erase(addr) && flash_page_program(addr, page_buf);
}

static uint8_t cmd_begin(const uint8_t *arg)
{
    uint32_t size = get32(&arg[0]);
    if (size == 0 || size > APP_MAX_SIZE || (size & 3))
        return IAP_ST_BAD_ARG;
    active = false;
    if (!flash_page_erase(APP_HDR_ADDR))    /* 先让旧程序失效：之后任何中断都停在 BL */
        return IAP_ST_FLASH;
    app_size = size;
    app_crc = get32(&arg[4]);
    app_ver = arg[8] | (arg[9] << 8);
    recv = 0;
    active = true;
    return ST_OK;
}

static uint8_t cmd_data(const uint8_t *arg)
{
    uint32_t off = get32(&arg[0]);
    uint8_t len = arg[4];
    if (!active || off != recv)
        return IAP_ST_BAD_STATE;
    if (len == 0 || len > IAP_DATA_MAX || recv + len > app_size)
        return IAP_ST_BAD_ARG;
    /* 一次 DATA（≤ 56 字节）可能跨页：逐段拷进页缓冲，满一页就擦除并编程 */
    const uint8_t *src = &arg[5];
    while (len)
    {
        uint32_t fill = recv % FLASH_PAGE_SIZE;
        uint32_t n = FLASH_PAGE_SIZE - fill;
        if (n > len)
            n = len;
        memcpy((uint8_t *)page_buf + fill, src, n);
        src += n;
        len -= n;
        recv += n;
        if (recv % FLASH_PAGE_SIZE == 0 && !flush_page(recv / FLASH_PAGE_SIZE - 1, FLASH_PAGE_SIZE))
        {
            active = false;
            return IAP_ST_FLASH;
        }
    }
    return ST_OK;
}

static uint8_t cmd_end(wr_t *w)
{
    if (!active || recv != app_size)
        return IAP_ST_BAD_STATE;
    uint32_t tail = recv % FLASH_PAGE_SIZE;
    if (tail && !flush_page(recv / FLASH_PAGE_SIZE, tail))
    {
        active = false;
        return IAP_ST_FLASH;
    }
    active = false;
    uint32_t crc = crc32_calc((const void *)APP_ADDR, app_size);
    put32(w, crc);
    if (crc != app_crc)
        return IAP_ST_CRC;
    iap_hdr_page(page_buf, app_size, app_crc, app_ver);
    return flash_page_program(APP_HDR_ADDR, page_buf) && iap_app_valid() ? ST_OK : IAP_ST_FLASH;
}

static uint8_t handle(const uint8_t *req, wr_t *w)
{
    const uint8_t *arg = &req[2];
    switch (req[0])
    {
    case CMD_SYS_INFO:
    {
        const iap_hdr_t *h = iap_hdr();
        bool ok = iap_hdr_ok(h);
        put8(w, 2);                     /* 2 = BL */
        put16(w, BL_VERSION);
        put8(w, IAP_PROTO);
        put8(w, ok && iap_app_valid());
        put16(w, ok ? h->version : 0);
        put32(w, ok ? h->size : 0);
        put32(w, APP_BASE);
        put32(w, APP_MAX_SIZE);
        return ST_OK;
    }
    case CMD_BL_BEGIN:
        return cmd_begin(arg);
    case CMD_BL_DATA:
        return cmd_data(arg);
    case CMD_BL_END:
        return cmd_end(w);
    case CMD_BL_BOOT:
        if (!iap_app_valid())
            return IAP_ST_NO_APP;
        boot_pending = true;
        return ST_OK;
    default:
        return IAP_ST_BAD_CMD;
    }
}

static void bl_process(void)
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

    if (boot_pending)
    {
        uint32_t t0 = millis();
        while (rsp_pending || usb_hid_tx_busy())
        {
            if (rsp_pending && usb_hid_send(rsp))
                rsp_pending = false;
            if (millis() - t0 > 100)
                break;
        }
        delay_ms(20);           /* 让主机把应答读走再复位 */
        NVIC_SystemReset();     /* 复位后 BL 看到有效程序、没有标志，直接跳转 */
        while (1)
            ;
    }
}

/*
 * 启动代码（handle_reset）依次：写 mstatus（MPP=0）→ 调用 SystemInit → mret 进入 main。也就是说 main 运行在用户模式，
 * 而 APP 的启动代码要写 mtvec、CORECFGR 等机器模式 CSR，在用户模式下会触发异常而卡死。
 * 所以“是否跳转 APP”在 SystemInit 里决定：这时还在机器模式、时钟与 Flash 仍是复位状态，
 * 直接跳过去，APP 的启动过程与正常复位后完全相同。BL 用 -Wl,--wrap=SystemInit 把启动代码里的 SystemInit 换成下面的函数；
 * 留在 BL 模式才调用真正的 SystemInit（切 PLL）。
 */
void __real_SystemInit(void);
void __wrap_SystemInit(void)
{
    gate_off();

    if (iap_flag_set())
    {
        start_rear = ((const volatile uint32_t *)BL_FLAG_ADDR)[1] == IAP_FLAG_REAR;
        flash_page_erase(BL_FLAG_ADDR);         /* 只进一次：下次复位回到程序 */
    }
#ifdef BL_SKIP_CRC      /* 排查用：只看程序头，不算整体 CRC */
    else if (iap_hdr_ok(iap_hdr()))
#else
    else if (iap_app_valid())
#endif
    {
        __disable_irq();
        ((void (*)(void))APP_BASE)();           /* 0x0000_1800 别名，与 APP 的链接地址一致；不返回 */
    }
    __real_SystemInit();
}

int main(void)
{
    SystemCoreClockUpdate();
    timebase_init();
    if (start_rear)
        usb_hid_route(true);
    else
        usb_hid_init();
    while (1)
    {
        bl_process();
        usb_hid_poll_route();
    }
}
