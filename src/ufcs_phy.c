#include "ufcs_phy.h"
#include "board.h"
#include "timebase.h"

#define PIN_DP              GPIO_Pin_0          /* PB0：充电设备 RX（D+） */
#define PIN_DM              GPIO_Pin_1          /* PB1：充电设备 TX（D−） */

#define EXTEN_WR_LOCK       (1u << 15)

/* TIM3 自由计数 12MHz（48MHz/4）：训练字节位宽 115200 ≈ 104 计数 */
#define TICK_HZ             12000000u
#define TICKS_US(us)        ((uint16_t)((us) * (TICK_HZ / 1000000u)))

/* 三个标称波特率：位宽（12MHz 计数）与 UART BRR（48MHz 计数） */
static const uint16_t nominal_ticks[3] = {104, 208, 312};
static const uint16_t nominal_brr[3]   = {417, 833, 1250};

/* 训练字节判定：位宽与标称相差超过 ±20% 视为波特率错误（规范 6.4.6） */
#define TRAIN_TOL_DIV       5

#define RX_RING             3
#define STOP_MASK           UART_CTLR2_STOP
#define STOP_2              UART_CTLR2_STOP_1

typedef enum
{
    RX_OFF,
    RX_IDLE,            /* 等训练字节起始位（EXTI 下降沿） */
    RX_TRAIN,           /* 收训练字节的 8 个边沿 */
    RX_BYTES,           /* UART 收其余字节 */
    RX_RESET_CHECK,     /* 训练字节中途停住且 D+ 为低：再观察一会儿，确认没有边沿才算硬复位 */
    RX_RESET_WAIT,      /* 判为硬复位，等 D+ 回高 */
} rx_state_t;

static struct
{
    volatile rx_state_t st;
    volatile uint8_t    edge;
    uint16_t            t[8];
    uint16_t            bit_ticks;
    uint8_t             baud;
    uint8_t             buf[UFCS_MAX_PKT];
    volatile uint8_t    n;
    uint8_t             want;
    volatile uint16_t   last_t;
    ufcs_rx_pkt_t       ring[RX_RING];
    volatile uint8_t    head, tail;
} rx;

static struct
{
    volatile bool     busy;
    uint8_t           buf[UFCS_MAX_PKT + 1];
    volatile uint8_t  n, i;
    volatile uint32_t end_us;
    uint32_t          start_ms;
} tx;

static ufcs_phy_stats_t stats;
static void (*yield_fn)(void);

void ufcs_phy_set_yield(void (*fn)(void))
{
    yield_fn = fn;
}

/* 带回调的等待：阻塞的握手/扫描期间也要喂看门狗 */
static void wait_us(uint32_t us)
{
    uint32_t t0 = micros();
    while ((uint32_t)(micros() - t0) < us)
    {
        if (yield_fn)
            yield_fn();
    }
}
#define delay_us(x) wait_us(x)
#define delay_ms(x) wait_us((uint32_t)(x) * 1000u)
static bool active;
static uint8_t tx_baud;
static volatile bool reset_seen;
static uint16_t reset_t0;
static uint16_t reset_len_us;   /* 硬复位低电平持续时间（µs，封顶 65535），读后清零 */

void EXTI7_0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/* ---------------- EXTEN：D+/D− 模拟功能 ---------------- */

static void exten1_modify(uint32_t clr, uint32_t set)
{
    EXTEN->EXTEN_KEYR = EXTEN_KEY1;
    EXTEN->EXTEN_KEYR = EXTEN_KEY2;
    EXTEN->EXTEN_CTLR1 = (EXTEN->EXTEN_CTLR1 & ~clr) | set;
    EXTEN->EXTEN_CTLR0 |= EXTEN_WR_LOCK;
}

/* UDP/UDM 的 DAC/比较器/上下拉全部关闭，回到普通 GPIO（保留输入使能） */
#define UDP_ANALOG_MASK (EXTEN_UDP_BUFOE | EXTEN_UDP_PCS | EXTEN_UDP_PUE | EXTEN_UDP_PDE | EXTEN_UDP_DAC | EXTEN_UDP_AE)
#define UDM_ANALOG_MASK (EXTEN_UDM_BUFOE | EXTEN_UDM_PCS | EXTEN_UDM_PUE | EXTEN_UDM_PDE | EXTEN_UDM_DAC | EXTEN_UDM_AE)

static void analog_off(void)
{
    exten1_modify(UDP_ANALOG_MASK | UDM_ANALOG_MASK | EXTEN_UDU_SHRT, 0);
}

static void pins_init_clock(void)
{
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOB | RCC_PB2Periph_AFIO, ENABLE);
}

static void pin_dp(GPIOMode_TypeDef mode)
{
    GPIO_InitTypeDef g = {0};
    g.GPIO_Pin = PIN_DP;
    g.GPIO_Speed = GPIO_Speed_30MHz;
    g.GPIO_Mode = mode;
    GPIO_Init(GPIOB, &g);
}

static void pin_dm(GPIOMode_TypeDef mode)
{
    GPIO_InitTypeDef g = {0};
    g.GPIO_Pin = PIN_DM;
    g.GPIO_Speed = GPIO_Speed_30MHz;
    g.GPIO_Mode = mode;
    GPIO_Init(GPIOB, &g);
}

static inline bool dp_high(void)
{
    return (GPIOB->INDR & PIN_DP) != 0;
}

/* ---------------- DCP 判别 ---------------- */

bool ufcs_phy_dcp_detect(uint8_t *flags)
{
    pins_init_clock();
    pin_dp(GPIO_Mode_IN_FLOATING);
    pin_dm(GPIO_Mode_IN_FLOATING);
    analog_off();
    delay_us(200);

    uint8_t f = 0;
    /* UDM：带缓冲的 DAC 输出 VDM_SRC ≈ 0.6V（12/64 × 3.3V = 0.62V）
     * UDP：80µA 下拉（IDP_SINK）+ 比较器，阈值 6/64 × 3.3V = 0.31V（VDAT_REF 0.25~0.4V），AI=1 表示 D+ 低于阈值 */
    exten1_modify(UDP_ANALOG_MASK | UDM_ANALOG_MASK,
                  EXTEN_UDM_PDE | EXTEN_UDM_PUE | EXTEN_UDM_AE | EXTEN_UDM_BUFOE | (12u << 21) |
                  EXTEN_UDP_PDE | EXTEN_UDP_PUE | EXTEN_UDP_AE | (6u << 5) | EXTEN_UDP_PCS_0);
    delay_ms(10);

    uint8_t high = 0;
    for (uint8_t i = 0; i < 4; i++)
    {
        if (!(EXTEN->EXTEN_CTLR1 & EXTEN_UDP_AI))
            high++;
        delay_us(500);
    }
    if (high >= 3)
        f |= 1;

    /* 撤去 D− 驱动：D+ 应回到低（区分 D+ 本来就悬空为高的情况） */
    exten1_modify(EXTEN_UDM_BUFOE | EXTEN_UDM_AE | EXTEN_UDM_PDE | EXTEN_UDM_PUE, 0);
    delay_ms(2);
    if (EXTEN->EXTEN_CTLR1 & EXTEN_UDP_AI)
        f |= 2;

    analog_off();
    if (flags)
        *flags = f;
    return (f & 1) != 0;
}

/* pin_dm_side = true 扫 UDM，否则扫 UDP；返回第一个使比较器输出 1（引脚 < DAC）的档位，64 = 高于满量程 */
static uint8_t comp_level(bool dm)
{
    for (uint8_t k = 1; k < 64; k++)
    {
        if (dm)
            exten1_modify(EXTEN_UDM_DAC, (uint32_t)k << 21);
        else
            exten1_modify(EXTEN_UDP_DAC, (uint32_t)k << 5);
        delay_us(30);
        if (EXTEN->EXTEN_CTLR1 & (dm ? EXTEN_UDM_AI : EXTEN_UDP_AI))
            return k;
    }
    return 64;
}

void ufcs_phy_scan(uint8_t lv[4])
{
    pins_init_clock();
    pin_dp(GPIO_Mode_IN_FLOATING);
    pin_dm(GPIO_Mode_IN_FLOATING);
    analog_off();
    delay_ms(1);

    const uint32_t dp_comp = EXTEN_UDP_PDE | EXTEN_UDP_PUE | EXTEN_UDP_AE;
    const uint32_t dm_comp = EXTEN_UDM_PDE | EXTEN_UDM_PUE | EXTEN_UDM_AE;
    const uint32_t dp_drive = dp_comp | EXTEN_UDP_BUFOE | (12u << 5);
    const uint32_t dm_drive = dm_comp | EXTEN_UDM_BUFOE | (12u << 21);

    exten1_modify(UDP_ANALOG_MASK, dp_comp);
    lv[0] = comp_level(false);
    analog_off();
    exten1_modify(UDM_ANALOG_MASK, dm_comp);
    lv[1] = comp_level(true);
    analog_off();

    exten1_modify(UDP_ANALOG_MASK | UDM_ANALOG_MASK, dm_drive | dp_comp);
    delay_ms(5);
    lv[2] = comp_level(false);
    analog_off();
    exten1_modify(UDP_ANALOG_MASK | UDM_ANALOG_MASK, dp_drive | dm_comp);
    delay_ms(5);
    lv[3] = comp_level(true);
    analog_off();
}

/* ---------------- 握手 ---------------- */

/* 切到 UART：D− 交给 UART1 TX（空闲高），D+ 作为 RX */
static void enter_uart(void)
{
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_USART1 | RCC_PB2Periph_GPIOB | RCC_PB2Periph_AFIO, ENABLE);
    RCC_PB1PeriphClockCmd(RCC_PB1Periph_TIM3, ENABLE);

    TIM3->CTLR1 = 0;
    TIM3->PSC = SystemCoreClock / TICK_HZ - 1;
    TIM3->ATRLR = 0xFFFF;
    TIM3->SWEVGR = 1;
    TIM3->CTLR1 = 1;

    tx_baud = UFCS_BAUD_115200;
    USART1->CTLR1 = 0;
    USART1->CTLR2 = 0;
    USART1->CTLR3 = 0;
    USART1->BRR = nominal_brr[tx_baud];
    USART1->CTLR1 = UART_CTLR1_UE | UART_CTLR1_TE;      /* TX 空闲为高，之后再把引脚交给它 */

    GPIO_PinRemapConfig(GPIO_PartialRemap3_USART1, ENABLE);     /* UART1_RM = 011：TX = PB1，RX = PB0 */
    pin_dp(GPIO_Mode_IN_FLOATING);
    pin_dm(GPIO_Mode_AF_PP);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);
    EXTI->INTENR &= ~1u;
    EXTI->RTENR &= ~1u;
    EXTI->FTENR &= ~1u;
    EXTI->INTFR = 1;
    NVIC_SetPriority(EXTI7_0_IRQn, 0x10);
    NVIC_SetPriority(USART1_IRQn, 0x40);
    NVIC_EnableIRQ(EXTI7_0_IRQn);
    NVIC_EnableIRQ(USART1_IRQn);

    memset(&rx, 0, sizeof(rx));
    memset(&tx, 0, sizeof(tx));
    reset_seen = false;
    active = true;
    rx.st = RX_OFF;
    /* rx_arm 在下面定义，这里直接内联 */
    USART1->CTLR1 &= ~(UART_CTLR1_RE | UART_CTLR1_RXNEIE);
    EXTI->INTFR = 1;
    EXTI->RTENR &= ~1u;
    EXTI->FTENR |= 1u;
    EXTI->INTENR |= 1u;
    rx.st = RX_IDLE;
}

bool ufcs_phy_handshake(uint8_t *tries)
{
    pins_init_clock();
    analog_off();
    pin_dp(GPIO_Mode_IN_FLOATING);
    GPIOB->BSHR = PIN_DM;
    pin_dm(GPIO_Mode_Out_PP);

    for (uint8_t attempt = 0; attempt < 3; attempt++)
    {
        if (tries)
            *tries = attempt + 1;

        GPIOB->BSHR = PIN_DM;               /* tDet1：高 2ms */
        delay_us(2000);
        GPIOB->BCR = PIN_DM;                /* tDet2：低 8ms */
        delay_us(8000);
        GPIOB->BSHR = PIN_DM;               /* tDet3：高 2ms */
        delay_us(2000);
        GPIOB->BCR = PIN_DM;                /* tDet4：低，同时监视 D+ 拉高，窗口 tDpDet 11~15ms */

        uint32_t t0 = micros();
        bool ok = false;
        while ((uint32_t)(micros() - t0) < 15000)
        {
            if (dp_high())
            {
                delay_us(100);              /* 确认不是毛刺 */
                if (dp_high())
                {
                    ok = true;
                    break;
                }
            }
        }
        if (ok)
        {
            /* 规范 tDet4 ≥ 6ms：D+ 拉高时若还不到 6ms，补足后再切换 */
            while ((uint32_t)(micros() - t0) < 6000)
                ;
            enter_uart();
            return true;
        }
        pin_dm(GPIO_Mode_IN_FLOATING);      /* 失败：高阻，tdetRetry 内重来 */
        delay_ms(5);
        GPIOB->BCR = PIN_DM;
        pin_dm(GPIO_Mode_Out_PP);
    }
    pin_dm(GPIO_Mode_IN_FLOATING);
    return false;
}

void ufcs_phy_release(void)
{
    if (active)
    {
        NVIC_DisableIRQ(EXTI7_0_IRQn);
        NVIC_DisableIRQ(USART1_IRQn);
        EXTI->INTENR &= ~1u;
        USART1->CTLR1 = 0;
        GPIO_PinRemapConfig(GPIO_PartialRemap3_USART1, DISABLE);
        TIM3->CTLR1 = 0;
        active = false;
    }
    pin_dp(GPIO_Mode_IN_FLOATING);
    pin_dm(GPIO_Mode_IN_FLOATING);
    analog_off();
}

bool ufcs_phy_active(void)
{
    return active;
}

/* ---------------- 接收 ---------------- */

static void rx_arm(void)
{
    USART1->CTLR1 &= ~(UART_CTLR1_RE | UART_CTLR1_RXNEIE);
    EXTI->INTFR = 1;
    EXTI->RTENR &= ~1u;
    EXTI->FTENR |= 1u;
    EXTI->INTENR |= 1u;
    rx.st = RX_IDLE;
}

static void rx_abort(void)
{
    rx_arm();
}

/* 8 个边沿（起始位下降沿 0，随后 2T 上升、3T 下降、4T 上升、5T 下降、6T 上升、7T 下降、8T 上升）算出位宽 */
static const uint8_t train_edges[8] = {0, 2, 3, 4, 5, 6, 7, 8};

static bool train_finish(void)
{
    uint16_t span = rx.t[7] - rx.t[0];
    uint16_t T = span >> 3;
    uint8_t cls = T < 146 ? 0 : T < 260 ? 1 : 2;
    uint16_t nom = nominal_ticks[cls];
    uint16_t tol = nom / TRAIN_TOL_DIV;
    if (T + tol < nom || T > nom + tol)
        return false;
    for (uint8_t k = 1; k < 7; k++)
    {
        int16_t d = (int16_t)(rx.t[k] - rx.t[0]) - (int16_t)(train_edges[k] * T);
        if (d < 0)
            d = -d;
        if (d > (int16_t)(T / 2))
            return false;
    }
    rx.bit_ticks = T;
    rx.baud = cls;
    return true;
}

void EXTI7_0_IRQHandler(void)
{
    EXTI->INTFR = 1;
    uint16_t now = TIM3->CNT;
    bool high = dp_high();

    if (rx.st == RX_IDLE)
    {
        if (high)
            return;                     /* 上升沿（多为上一包的尾巴），忽略 */
        rx.t[0] = now;
        rx.edge = 1;
        rx.st = RX_TRAIN;
        EXTI->RTENR |= 1u;
        EXTI->FTENR |= 1u;
        return;
    }
    if (rx.st == RX_TRAIN)
    {
        /* 边沿 k 之后的电平：奇数号为高、偶数号为低 */
        if (high != (bool)(rx.edge & 1))
        {
            stats.train_bad++;
            rx_arm();
            return;
        }
        rx.t[rx.edge++] = now;
        if (rx.edge < 8)
            return;
        if (!train_finish())
        {
            stats.train_bad++;
            rx_arm();
            return;
        }
        stats.train_ok++;
        /* 用实测位宽接收其余字节：BRR = 位宽的 48MHz 计数 */
        USART1->BRR = (uint16_t)(rx.bit_ticks * (SystemCoreClock / TICK_HZ));
        USART1->CTLR2 &= ~STOP_MASK;
        (void)USART1->STATR;
        (void)USART1->DATAR;
        rx.n = 0;
        rx.want = 0;
        rx.last_t = now;
        rx.st = RX_BYTES;
        EXTI->INTENR &= ~1u;
        USART1->CTLR1 |= UART_CTLR1_RE | UART_CTLR1_RXNEIE;
        return;
    }
    if (rx.st == RX_RESET_CHECK)
    {
        rx_arm();               /* 又有边沿：D+ 在动，不是硬复位 */
        stats.train_bad++;
    }
}

static void rx_done(void)
{
    uint8_t next = (rx.head + 1) % RX_RING;
    if (next == rx.tail)
    {
        stats.overflow++;
    }
    else
    {
        ufcs_rx_pkt_t *p = &rx.ring[rx.head];
        p->len = rx.n;
        p->baud = rx.baud;
        p->bit_ticks = rx.bit_ticks;
        p->t_us = micros();
        memcpy(p->data, rx.buf, rx.n);
        rx.head = next;
    }
    rx_arm();
}

void USART1_IRQHandler(void)
{
    uint16_t sr = USART1->STATR;

    if ((sr & (UART_STATR_FE | UART_STATR_ORE | UART_STATR_NE)) && rx.st == RX_BYTES)
    {
        (void)USART1->DATAR;
        stats.frame_err++;
        rx_abort();
        return;
    }
    if ((sr & UART_STATR_RXNE) && rx.st == RX_BYTES)
    {
        uint8_t b = (uint8_t)USART1->DATAR;
        rx.last_t = TIM3->CNT;
        rx.buf[rx.n++] = b;
        if (rx.want == 0 || rx.want == 0xFF)
        {
            rx.want = ufcs_frame_len(rx.buf, rx.n);
            if (rx.want == 0xFF)
            {
                stats.frame_err++;
                rx_abort();
                return;
            }
        }
        if (rx.want && rx.n >= rx.want)
            rx_done();
        else if (rx.n >= UFCS_MAX_PKT)
            rx_abort();
        return;
    }
    if ((sr & UART_STATR_TXE) && (USART1->CTLR1 & UART_CTLR1_TXEIE))
    {
        if (tx.i < tx.n)
        {
            USART1->DATAR = tx.buf[tx.i++];
        }
        else
        {
            USART1->CTLR1 = (USART1->CTLR1 & ~UART_CTLR1_TXEIE) | UART_CTLR1_TCIE;
        }
        return;
    }
    if ((sr & UART_STATR_TC) && (USART1->CTLR1 & UART_CTLR1_TCIE))
    {
        USART1->CTLR1 &= ~UART_CTLR1_TCIE;
        USART1->CTLR2 &= ~STOP_MASK;
        tx.end_us = micros();
        tx.busy = false;
        rx_arm();
        return;
    }
    /* 其它标志（例如关闭 RE 后残留的 RXNE）：读数据清掉 */
    if (sr & UART_STATR_RXNE)
        (void)USART1->DATAR;
}

bool ufcs_phy_recv(ufcs_rx_pkt_t *p)
{
    if (rx.tail == rx.head)
        return false;
    *p = rx.ring[rx.tail];
    rx.tail = (rx.tail + 1) % RX_RING;
    return true;
}

bool ufcs_phy_rx_busy(void)
{
    return rx.st == RX_TRAIN || rx.st == RX_BYTES;
}

void ufcs_phy_process(void)
{
    if (!active)
        return;

    /* 发送不会超过 (64+1) × 11 位 / 38400 ≈ 19ms；超时说明 TC 中断丢了，强制恢复 */
    if (tx.busy && millis() - tx.start_ms > 40)
    {
        NVIC_DisableIRQ(USART1_IRQn);
        NVIC_DisableIRQ(EXTI7_0_IRQn);
        USART1->CTLR1 &= ~(UART_CTLR1_TXEIE | UART_CTLR1_TCIE);
        USART1->CTLR2 &= ~STOP_MASK;
        tx.busy = false;
        tx.end_us = micros();
        rx_arm();
        NVIC_EnableIRQ(EXTI7_0_IRQn);
        NVIC_EnableIRQ(USART1_IRQn);
    }

    uint16_t now = TIM3->CNT;
    if (rx.st == RX_BYTES && (uint16_t)(now - rx.last_t) > TICKS_US(1000))
    {
        NVIC_DisableIRQ(USART1_IRQn);
        if (rx.st == RX_BYTES)
        {
            stats.timeout++;
            rx_abort();
        }
        NVIC_EnableIRQ(USART1_IRQn);
    }
    else if (rx.st == RX_TRAIN && (uint16_t)(now - rx.t[rx.edge - 1]) > TICKS_US(900))
    {
        /* 训练字节中途停住（数据里最长的低电平只有 0.23ms）：D+ 为低则进入确认，否则是干扰 */
        NVIC_DisableIRQ(EXTI7_0_IRQn);
        if (rx.st == RX_TRAIN)
        {
            if (!dp_high())
            {
                reset_t0 = rx.t[rx.edge - 1];
                rx.st = RX_RESET_CHECK;
            }
            else
            {
                stats.train_bad++;
                rx_arm();
            }
        }
        NVIC_EnableIRQ(EXTI7_0_IRQn);
    }
    else if (rx.st == RX_RESET_CHECK)
    {
        NVIC_DisableIRQ(EXTI7_0_IRQn);
        if (rx.st == RX_RESET_CHECK)
        {
            if (dp_high())
            {
                stats.train_bad++;
                rx_arm();
            }
            else if ((uint16_t)(now - reset_t0) > TICKS_US(1500))
            {
                /* 规范 tResetSource ≥ 2ms：低电平 1.5ms 仍无任何边沿，确认为硬复位 */
                stats.hard_reset_rx++;
                reset_seen = true;
                rx.st = RX_RESET_WAIT;
                EXTI->INTENR &= ~1u;
            }
        }
        NVIC_EnableIRQ(EXTI7_0_IRQn);
    }

    if (rx.st == RX_RESET_WAIT && dp_high())
    {
        uint32_t us = (uint16_t)(now - reset_t0) / (TICK_HZ / 1000000u);
        reset_len_us = us ? (uint16_t)us : 1;
        rx_arm();
    }
}

uint16_t ufcs_phy_take_reset_len(void)
{
    uint16_t v = reset_len_us;
    reset_len_us = 0;
    return v;
}

bool ufcs_phy_take_reset(void)
{
    bool r = reset_seen;
    reset_seen = false;
    return r;
}

/* ---------------- 发送 ---------------- */

void ufcs_phy_set_baud(uint8_t baud)
{
    tx_baud = baud > 2 ? 0 : baud;
}

uint8_t ufcs_phy_baud(void)
{
    return tx_baud;
}

bool ufcs_phy_send(const uint8_t *pkt, uint8_t len)
{
    if (!active || tx.busy || len == 0 || len > UFCS_MAX_PKT)
        return false;
    NVIC_DisableIRQ(EXTI7_0_IRQn);
    NVIC_DisableIRQ(USART1_IRQn);
    if (rx.st != RX_IDLE || tx.busy)
    {
        NVIC_EnableIRQ(USART1_IRQn);
        NVIC_EnableIRQ(EXTI7_0_IRQn);
        return false;
    }
    EXTI->INTENR &= ~1u;
    tx.buf[0] = 0xAA;
    memcpy(&tx.buf[1], pkt, len);
    tx.n = len + 1;
    tx.i = 0;
    tx.busy = true;
    tx.start_ms = millis();
    rx.st = RX_OFF;
    USART1->CTLR1 &= ~(UART_CTLR1_RE | UART_CTLR1_RXNEIE);
    USART1->BRR = nominal_brr[tx_baud];
    USART1->CTLR2 = (USART1->CTLR2 & ~STOP_MASK) | STOP_2;     /* 8N2：帧间至少 1 位高电平（规范 6.4.7.1） */
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_EnableIRQ(EXTI7_0_IRQn);
    USART1->CTLR1 |= UART_CTLR1_TXEIE;
    return true;
}

bool ufcs_phy_tx_busy(void)
{
    return tx.busy;
}

uint32_t ufcs_phy_tx_end_us(void)
{
    return tx.end_us;
}

/* 充电设备硬复位：D− 拉低 ≥ 2ms（规范表 12 tResetSink），随后回到 UART 空闲态 */
void ufcs_phy_hard_reset(void)
{
    if (!active)
        return;
    while (tx.busy)
        ;
    NVIC_DisableIRQ(EXTI7_0_IRQn);
    NVIC_DisableIRQ(USART1_IRQn);
    GPIO_PinRemapConfig(GPIO_PartialRemap3_USART1, DISABLE);
    GPIOB->BCR = PIN_DM;
    pin_dm(GPIO_Mode_Out_PP);
    delay_us(2500);
    GPIOB->BSHR = PIN_DM;
    GPIO_PinRemapConfig(GPIO_PartialRemap3_USART1, ENABLE);
    pin_dm(GPIO_Mode_AF_PP);
    rx.head = rx.tail = 0;
    rx_arm();
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_EnableIRQ(EXTI7_0_IRQn);
}

const ufcs_phy_stats_t *ufcs_phy_stats(void)
{
    return &stats;
}
