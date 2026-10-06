/*
 * HiSilicon D− 单线物理层（AFC / FCP / SCP 共用；BC1.2/DCP 握手后 D+ 保持 0.6V，D− 双向，只由设备端发起）
 *   UI = 160µs；主机 Ping = D− 高 16 UI；从机 Ping 高 10~20 UI（实测 16 UI）
 *   字节：同步 = 1/4 UI 高-低-高（第三段可与数据位 1 相连）→ 8 位 MSB 先（每位 1 UI，高 = 1）→ 奇校验位
 *   发送时序照三星开源内核 drivers/afc/gpio_afc.c；从机回应格式由探测固件实测（Sping → 约 4 UI 低 → 字节… → Sping）
 * D− 发送用 UDM 缓冲 DAC（0 / 63 ≈ 3.25V），接收用 UDM 比较器（阈值 20/64 × 3.3V ≈ 1.03V）。
 * 发送一个字节约 1.8ms，期间关中断（USBPD 中断会打乱位宽）；接收靠轮询比较器记录跳变，中断只会让个别跳变时刻偏晚，由重试兜底。
 */
#include "hisi_phy.h"
#include "board.h"
#include "timebase.h"

#define EXTEN_WR_LOCK   (1u << 15)
#define DM_MASK         (EXTEN_UDM_BUFOE | EXTEN_UDM_PCS | EXTEN_UDM_PUE | EXTEN_UDM_PDE | EXTEN_UDM_DAC | EXTEN_UDM_AE)
#define DM_DRIVE(k)     (EXTEN_UDM_PDE | EXTEN_UDM_PUE | EXTEN_UDM_AE | EXTEN_UDM_BUFOE | ((uint32_t)(k) << 21))
#define DM_COMP(k)      (EXTEN_UDM_PDE | EXTEN_UDM_PUE | EXTEN_UDM_AE | ((uint32_t)(k) << 21))
#define LV_HIGH         63u
#define CMP_RX          20u

uint16_t hp_edges[HP_MAX_EDGES];
uint8_t hp_n_edges, hp_first_level;

static void exten1_modify(uint32_t clr, uint32_t set)
{
    EXTEN->EXTEN_KEYR = EXTEN_KEY1;
    EXTEN->EXTEN_KEYR = EXTEN_KEY2;
    EXTEN->EXTEN_CTLR1 = (EXTEN->EXTEN_CTLR1 & ~clr) | set;
    EXTEN->EXTEN_CTLR0 |= EXTEN_WR_LOCK;
}

void hp_out(bool high)
{
    exten1_modify(DM_MASK, DM_DRIVE(high ? LV_HIGH : 0));
}

static void rx_mode(void)
{
    exten1_modify(DM_MASK, DM_COMP(CMP_RX));
}

/* 比较器：AI = 1 表示 D− 低于阈值 */
static bool rx_high(void)
{
    return (EXTEN->EXTEN_CTLR1 & EXTEN_UDM_AI) == 0;
}

/* 直接数 SysTick（关中断时 micros() 只能补一次回卷，跨两个 ms 会算错） */
static void wait_us(uint32_t us)
{
    uint32_t need = us * (SystemCoreClock / 1000000), top = SysTick->CMP + 1, prev = SysTick->CNT, acc = 0;
    while (acc < need)
    {
        uint32_t c = SysTick->CNT;
        acc += c >= prev ? c - prev : c + top - prev;
        prev = c;
    }
}

static void cycle(uint32_t us)
{
    hp_out(true);
    wait_us(us);
    hp_out(false);
    wait_us(us);
}

void hp_mping(void)
{
    hp_out(true);
    wait_us(16 * HP_UI);
    hp_out(false);
}

int32_t hp_sping(void)
{
    rx_mode();
    uint32_t s = micros();
    while (!rx_high())
        if (micros() - s > 6 * HP_UI)
            return -1;
    uint32_t h = micros();
    while (rx_high())
        if (micros() - h > 21 * HP_UI)
            return -2;
    uint32_t len = micros() - h;
    return len < 10 * HP_UI ? -3 : (int32_t)len;
}

void hp_send_byte(uint8_t d)
{
    __disable_irq();
    hp_out(false);
    wait_us(HP_UI);
    cycle(HP_UI / 4);
    if (!(d & 0x80))
    {
        hp_out(true);
        wait_us(HP_UI / 4);
    }
    for (uint8_t m = 0x80; m; m >>= 1)
    {
        hp_out(d & m);
        wait_us(HP_UI);
    }
    uint8_t ones = 0;
    for (uint8_t v = d; v; v >>= 1)
        ones += v & 1;
    bool odd = ones & 1;
    hp_out(!odd);               /* 奇校验：数据 + 校验位共奇数个 1（同三星 gpio_afc.c） */
    wait_us(HP_UI);
    if (odd)
        hp_out(false);
    wait_us(HP_UI / 4);
    cycle(HP_UI / 4);
    __enable_irq();
}

void hp_capture(uint32_t ms)
{
    rx_mode();
    delay_us(5);
    hp_n_edges = 0;
    bool lv = rx_high();
    hp_first_level = lv;
    uint32_t s = micros(), last = s;
    while (micros() - s < ms * 1000 && hp_n_edges < HP_MAX_EDGES)
    {
        bool v = rx_high();
        if (v != lv)
        {
            uint32_t now = micros();
            hp_edges[hp_n_edges++] = (uint16_t)(now - last);
            last = now;
            lv = v;
        }
    }
}

static bool level_at(uint32_t t)
{
    uint32_t a = 0;
    bool lv = hp_first_level;
    for (uint8_t i = 0; i < hp_n_edges; i++)
    {
        if (t < a + hp_edges[i])
            return lv;
        a += hp_edges[i];
        lv = !lv;
    }
    return lv;
}

uint8_t hp_decode(uint8_t *out, uint8_t max, uint8_t *bad)
{
    uint32_t a = 0, skip = 0;
    uint8_t n = 0;
    bool lv = hp_first_level;
    *bad = 0;
    for (uint8_t i = 0; i + 2 < hp_n_edges && n < max; i++)
    {
        uint32_t len = hp_edges[i];
        /* 同步：短高 + 短低，且下一段不是从机 Ping（字节内最长的高电平 1/4 + 9 UI ≈ 1480µs） */
        if (a >= skip && lv && len < 80 && hp_edges[i + 1] < 80 && hp_edges[i + 2] < 1550)
        {
            uint32_t ds = a + len + hp_edges[i + 1] + HP_UI / 4;
            uint8_t b = 0, ones = 0;
            for (uint8_t k = 0; k < 8; k++)
            {
                bool bit = level_at(ds + k * HP_UI + HP_UI / 2);
                b = (uint8_t)((b << 1) | bit);
                ones += bit;
            }
            ones += level_at(ds + 8 * HP_UI + HP_UI / 2);
            if (!(ones & 1))
                *bad |= (uint8_t)(1u << n);
            out[n++] = b;
            skip = ds + 9 * HP_UI - 60;
        }
        a += len;
        lv = !lv;
    }
    return n;
}

uint8_t hp_xfer(const uint8_t *tx, uint8_t n, uint32_t cap_ms, int32_t *s0, int32_t *s1)
{
    hp_mping();
    *s0 = hp_sping();
    hp_n_edges = 0;
    if (*s0 >= 0)
    {
        for (uint8_t i = 0; i < n; i++)
            hp_send_byte(tx[i]);
        hp_mping();
        hp_capture(cap_ms);
        hp_out(false);
        delay_us(200);
        hp_mping();
    }
    *s1 = *s0 >= 0 ? hp_sping() : 0;
    hp_out(false);
    return hp_n_edges;
}
