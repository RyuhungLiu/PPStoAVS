/*
 * QC2.0 / QC3.0 前端诱骗探测固件（-DQC_PROBE）：上电在协议桥之前阻塞执行一次，结果写入事件记录（EV_QC_STEP）。
 *  1. D+ 由 UDP 的缓冲 DAC 输出 0.6V；D− 用 UDM 比较器（阈值 0.31V）看是否被充电器短接到 D+（DCP）
 *     不是 DCP（接电脑等）→ 放开引脚、返回 false，照常启动 USB HID
 *  2. 保持 D+ 0.6V，等充电器断开短接（D− 被下拉，QC 规范 ≥ 1.25s），最多 3s
 *  3. QC2：依次 9V（D+ 3.3 / D− 0.6）、12V（0.6 / 0.6）、20V（3.3 / 3.3）、5V（0.6 / 0），各自量 VBUS
 *  4. QC3：连续模式（0.6 / 3.3），用 1ms、0.2ms、5ms 三种脉宽各升 5 步、降 5 步（每步 ±200mV），量 VBUS
 *  5. 回到 5V 并保持（D+ 0.6V），返回 true：主循环只写记录
 * 电平全部用 UDP/UDM 的 6 位 DAC 缓冲输出（DAC/64 × 3.3V）：0V = 0，0.6V = 12，3.3V = 63（≈3.25V）。后端开关保持关断。
 */
#ifdef QC_PROBE

#include "qc_probe.h"
#include "analog.h"
#include "board.h"
#include "evlog.h"
#include "timebase.h"

#define LV_0V       0u
#define LV_0V6      12u
#define LV_3V3      63u
#define EXTEN_WR_LOCK (1u << 15)    /* 同 ufcs_phy.c：EXTEN_CTLR0 写锁 */
#define CMP_0V31    6u              /* 比较器阈值 6/64 × 3.3V ≈ 0.31V（VDAT_REF 0.25~0.4V） */

#define DP_MASK     (EXTEN_UDP_BUFOE | EXTEN_UDP_PCS | EXTEN_UDP_PUE | EXTEN_UDP_PDE | EXTEN_UDP_DAC | EXTEN_UDP_AE)
#define DM_MASK     (EXTEN_UDM_BUFOE | EXTEN_UDM_PCS | EXTEN_UDM_PUE | EXTEN_UDM_PDE | EXTEN_UDM_DAC | EXTEN_UDM_AE)
#define DP_DRIVE(k) (EXTEN_UDP_PDE | EXTEN_UDP_PUE | EXTEN_UDP_AE | EXTEN_UDP_BUFOE | ((uint32_t)(k) << 5))
#define DM_DRIVE(k) (EXTEN_UDM_PDE | EXTEN_UDM_PUE | EXTEN_UDM_AE | EXTEN_UDM_BUFOE | ((uint32_t)(k) << 21))
#define DM_COMP(k)  (EXTEN_UDM_PDE | EXTEN_UDM_PUE | EXTEN_UDM_AE | ((uint32_t)(k) << 21))

static uint32_t t0;

static void exten1_modify(uint32_t clr, uint32_t set)
{
    EXTEN->EXTEN_KEYR = EXTEN_KEY1;
    EXTEN->EXTEN_KEYR = EXTEN_KEY2;
    EXTEN->EXTEN_CTLR1 = (EXTEN->EXTEN_CTLR1 & ~clr) | set;
    EXTEN->EXTEN_CTLR0 |= EXTEN_WR_LOCK;
}

static void log_step(uint8_t code, uint8_t arg, uint16_t x, uint16_t vbus)
{
    ev_qc_step_t e = {code, arg, x, vbus, (uint16_t)(millis() - t0)};
    evlog_add(EV_QC_STEP, &e, sizeof(e));
}

/* 等待期间顺便把记录写进 Flash（每 50ms 至多一次页操作；DAC 电平由硬件保持） */
static void wait_ms(uint32_t ms)
{
    uint32_t s = millis(), last = 0;
    while (millis() - s < ms)
    {
        if (millis() - last >= 50 && evlog_flush_step())
            last = millis();
    }
}

static uint16_t vbus_avg(void)
{
    uint32_t sum = 0;
    for (uint8_t i = 0; i < 8; i++)
    {
        sum += analog_vbus_mv();
        delay_us(200);
    }
    return (uint16_t)(sum / 8);
}

static void set_lv(uint8_t dp, uint8_t dm)
{
    exten1_modify(DP_MASK | DM_MASK, DP_DRIVE(dp) | DM_DRIVE(dm));
}

/* D− 比较器：true = D− 低于 0.31V */
static bool dm_low(void)
{
    return (EXTEN->EXTEN_CTLR1 & EXTEN_UDM_AI) != 0;
}

/* D− 电平扫描（D+ 保持驱动），返回第一个让 D− < DAC 的档位，64 = 高于满量程 */
static uint8_t dm_level(void)
{
    for (uint8_t k = 1; k < 64; k++)
    {
        exten1_modify(EXTEN_UDM_DAC, (uint32_t)k << 21);
        delay_us(30);
        if (dm_low())
            return k;
    }
    return 64;
}

/* 不驱动时两脚的电平（比较器扫描）：看充电器自己在 D+/D− 上放了什么（DCP 短接、Apple/三星分压等） */
static void idle_scan(uint8_t *dp, uint8_t *dm)
{
    exten1_modify(DP_MASK | DM_MASK, (EXTEN_UDP_PDE | EXTEN_UDP_PUE | EXTEN_UDP_AE) | DM_COMP(0));
    delay_ms(2);
    *dp = 64;
    for (uint8_t k = 1; k < 64; k++)
    {
        exten1_modify(EXTEN_UDP_DAC, (uint32_t)k << 5);
        delay_us(30);
        if (EXTEN->EXTEN_CTLR1 & EXTEN_UDP_AI)
        {
            *dp = k;
            break;
        }
    }
    *dm = dm_level();
}

static void release(void)
{
    exten1_modify(DP_MASK | DM_MASK | EXTEN_UDU_SHRT, 0);
}

static void mode(uint8_t m, uint8_t dp, uint8_t dm, uint32_t settle)
{
    set_lv(dp, dm);
    wait_ms(settle);
    log_step(QCS_MODE, m, (uint16_t)settle, vbus_avg());
}

/* QC3 连续模式脉冲：up = D+ 0.6→3.3→0.6；down = D− 3.3→0.6→3.3 */
static void pulses(bool up, uint8_t n, uint16_t width_us)
{
    for (uint8_t i = 0; i < n; i++)
    {
        if (up)
            set_lv(LV_3V3, LV_3V3);
        else
            set_lv(LV_0V6, LV_0V6);
        delay_us(width_us);
        set_lv(LV_0V6, LV_3V3);
        delay_us(width_us);
    }
    wait_ms(150);
    log_step(QCS_PULSE, (uint8_t)((up ? 0x80 : 0) | n), width_us, vbus_avg());
}

bool qc_probe_boot(void)
{
    t0 = millis();
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOB, ENABLE);
    GPIO_InitTypeDef g = {0};
    g.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    g.GPIO_Speed = GPIO_Speed_30MHz;
    g.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &g);
    release();
    delay_ms(1);

    /* 0. 不驱动时的电平 */
    uint8_t ip, im;
    idle_scan(&ip, &im);
    log_step(QCS_IDLE, ip, im, vbus_avg());

    /* 1. D+ 0.6V，看 D− 是否跟随（DCP 短接）；有的充电器上电后才接上短接，最多看 3s */
    exten1_modify(DP_MASK | DM_MASK, DP_DRIVE(LV_0V6) | DM_COMP(CMP_0V31));
    static const uint16_t at[] = {20, 500, 1500, 3000};
    bool dcp = false;
    uint32_t s0 = millis();
    for (uint8_t i = 0; i < sizeof(at) / sizeof(at[0]) && !dcp; i++)
    {
        uint32_t el = millis() - s0;
        if (el < at[i])
            wait_ms(at[i] - el);
        uint8_t lv = dm_level();
        exten1_modify(EXTEN_UDM_DAC, (uint32_t)CMP_0V31 << 21);
        delay_us(30);
        dcp = !dm_low();
        log_step(QCS_DCP, dcp, lv, vbus_avg());
    }
    if (!dcp)
    {
        release();
        wait_ms(2500);          /* 等记录封页写入 Flash（2s 无新记录才封页），再交给 USB HID（D+ 1.5k 上拉 → 3.3V） */
        return false;
    }

    /* 2. 保持 D+ 0.6V，等 D− 被放开（连续 10ms 低于 0.31V） */
    uint32_t s = millis(), low_since = 0;
    bool released = false;
    while (millis() - s < 3000)
    {
        if (dm_low())
        {
            if (!low_since)
                low_since = millis();
            else if (millis() - low_since >= 10)
            {
                released = true;
                break;
            }
        }
        else
            low_since = 0;
    }
    log_step(QCS_BC_DONE, released, released ? (uint16_t)(low_since - s) : 0, vbus_avg());

    /* 3. QC2 定压（没看到放开也照试一次，记录充电器反应） */
    mode(QCM_5V, LV_0V6, LV_0V, 200);
    mode(QCM_9V, LV_3V3, LV_0V6, 400);
    mode(QCM_12V, LV_0V6, LV_0V6, 400);
    mode(QCM_20V, LV_3V3, LV_3V3, 400);
    mode(QCM_5V, LV_0V6, LV_0V, 400);

    /* 4. QC3 连续模式 */
    mode(QCM_CONT, LV_0V6, LV_3V3, 200);
    static const uint16_t widths[] = {1000, 200, 5000};
    for (uint8_t w = 0; w < sizeof(widths) / sizeof(widths[0]); w++)
    {
        pulses(true, 5, widths[w]);
        pulses(false, 5, widths[w]);
    }

    /* 5. 回 5V 并保持 */
    mode(QCM_5V, LV_0V6, LV_0V, 400);
    log_step(QCS_DONE, 0, 0, vbus_avg());
    return true;
}

void qc_probe_idle(void)
{
    static uint32_t last;
    if (millis() - last >= 50 && evlog_flush_step())
        last = millis();
}

#endif
