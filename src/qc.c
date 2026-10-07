/*
 * QC2.0 / QC3.0 前端（Sink）：用 UDP/UDM 的 6 位 DAC 缓冲输出给 D+/D− 电平（0V = 0，0.6V = 12，3.3V = 63 ≈ 3.25V）。
 * 实测（v0.13.0 探测固件）：D+ 0.6V 后约 1.07s 充电器放开 D−；QC2 9V/12V、QC3 ±200mV/脉冲（0.2~5ms 脉宽都有效）。
 *  - 握手：D+ 0.6V，D− 跟随（DCP 短接）→ 等 D− 被放开（≤ 3s）
 *  - 能力探测：先在 5V 进连续模式（等 200ms 去抖）升 3 步，VBUS 升高 > 300mV 即 QC3（兼容 QC2，认定 5/9/12V）；
 *    不是 QC3 才逐档量 9V、12V；最后回 5V。握手 + 探测约 1.6s（QC3）/ 2.1s（仅 QC2）
 *  - 连续模式切入后等 200ms（充电器去抖）再按实测 VBUS 定起点（不依赖充电器从哪个电压开始），每步 200mV，脉冲 1ms + 1ms，非阻塞
 */
#include "qc.h"
#include "analog.h"
#include "board.h"
#include "timebase.h"

#define EXTEN_WR_LOCK   (1u << 15)
#define LV_0V           0u
#define LV_0V6          12u
#define LV_3V3          63u
#define CMP_0V31        6u

#define DP_MASK     (EXTEN_UDP_BUFOE | EXTEN_UDP_PCS | EXTEN_UDP_PUE | EXTEN_UDP_PDE | EXTEN_UDP_DAC | EXTEN_UDP_AE)
#define DM_MASK     (EXTEN_UDM_BUFOE | EXTEN_UDM_PCS | EXTEN_UDM_PUE | EXTEN_UDM_PDE | EXTEN_UDM_DAC | EXTEN_UDM_AE)
#define DP_DRIVE(k) (EXTEN_UDP_PDE | EXTEN_UDP_PUE | EXTEN_UDP_AE | EXTEN_UDP_BUFOE | ((uint32_t)(k) << 5))
#define DM_DRIVE(k) (EXTEN_UDM_PDE | EXTEN_UDM_PUE | EXTEN_UDM_AE | EXTEN_UDM_BUFOE | ((uint32_t)(k) << 21))
#define DM_COMP(k)  (EXTEN_UDM_PDE | EXTEN_UDM_PUE | EXTEN_UDM_AE | ((uint32_t)(k) << 21))

#define T_PULSE_US      1000
#define T_FIXED_MS      200     /* QC2 换档：充电器去抖 20~60ms + 爬升 */
#define T_CONT_SETTLE   40      /* 连续模式最后一个脉冲后 */
#define T_CONT_ENTER    200     /* 进连续模式后等充电器去抖（规范 20~60ms，探测固件实测 200ms 可靠）再发脉冲 */
#define QC3_MIN_MV      3600

static void (*yield_fn)(void);
static bool cont;
static uint16_t cur_mv;
static int8_t steps;            /* 待发脉冲：> 0 升，< 0 降 */
static uint8_t phase;           /* 0 空闲，1 脉冲有效，2 脉冲间隔，3 等稳定 */
static uint32_t ts;
static uint32_t tu;             /* 脉冲相位的 µs 时间戳 */
static uint16_t settle;         /* phase 3 的等待 ms */
static uint16_t target;
static int16_t off;             /* 充电器输出偏差：5V 档实测 − 5000（实测约 +120mV），连续模式按标称值换算 */
static void start_steps(void);

void qc_set_yield(void (*fn)(void))
{
    yield_fn = fn;
}

static void exten1_modify(uint32_t clr, uint32_t set)
{
    EXTEN->EXTEN_KEYR = EXTEN_KEY1;
    EXTEN->EXTEN_KEYR = EXTEN_KEY2;
    EXTEN->EXTEN_CTLR1 = (EXTEN->EXTEN_CTLR1 & ~clr) | set;
    EXTEN->EXTEN_CTLR0 |= EXTEN_WR_LOCK;
}

static void set_lv(uint8_t dp, uint8_t dm)
{
    exten1_modify(DP_MASK | DM_MASK, DP_DRIVE(dp) | DM_DRIVE(dm));
}

static void wait_ms(uint32_t ms)
{
    uint32_t s = millis();
    while (millis() - s < ms)
        if (yield_fn)
            yield_fn();
}

static bool dm_low(void)
{
    return (EXTEN->EXTEN_CTLR1 & EXTEN_UDM_AI) != 0;
}

static uint16_t vbus(void)
{
    uint32_t s = 0;
    for (uint8_t i = 0; i < 8; i++)
        s += analog_vbus_mv();
    return (uint16_t)(s / 8);
}

/* QC2 定压档位 */
static void set_fixed(uint16_t mv)
{
    if (mv >= 12000)
        set_lv(LV_0V6, LV_0V6);
    else if (mv >= 9000)
        set_lv(LV_3V3, LV_0V6);
    else
        set_lv(LV_0V6, LV_0V);
    cont = false;
    cur_mv = mv >= 12000 ? 12000 : mv >= 9000 ? 9000 : 5000;
}

static void release(void)
{
    exten1_modify(DP_MASK | DM_MASK | EXTEN_UDU_SHRT, 0);
}

/* BC1.2 → HVDCP 握手（QC、AFC、FCP 共用）：D+ 0.6V，D− 跟随（DCP 短接）后等充电器放开；
 * 成功时 D+ 保持 0.6V、D− 留在比较器模式；失败时放开两脚。返回放开所用 ms，0 = 失败 */
uint16_t qc_handshake(void)
{
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOB, ENABLE);
    GPIO_InitTypeDef g = {0};
    g.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    g.GPIO_Speed = GPIO_Speed_30MHz;
    g.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &g);

    /* D+ 0.6V，D− 比较器 0.31V：先看跟随（短接），再等放开 */
    exten1_modify(DP_MASK | DM_MASK, DP_DRIVE(LV_0V6) | DM_COMP(CMP_0V31));
    uint32_t s = millis(), low = 0;
    bool seen = false;
    for (;;)
    {
        if (yield_fn)
            yield_fn();
        uint32_t now = millis();
        if (now - s > 3000)
        {
            release();
            return 0;
        }
        if (!dm_low())
        {
            seen = true;
            low = 0;
        }
        else if (seen)
        {
            if (!low)
                low = now;
            else if (now - low >= 10)
                return (uint16_t)(low - s) | 1;
        }
        else if (now - s > 500)
        {
            release();          /* D− 一直不跟随：不是 DCP */
            return 0;
        }
    }
}

void qc_release(void)
{
    release();
}

void qc_lines_low(void)
{
    set_lv(LV_0V, LV_0V);
}

uint8_t qc_detect(uint16_t *dv, uint16_t *base)
{
    *dv = 0;
    *base = 0;

    /* 先在 5V 测 QC3（不用等高压放电）：QC3 向下兼容 QC2，测到就认定 5/9/12V 都有；没有 QC3 才逐档测 QC2 */
    uint8_t f = QC_OK;
    set_fixed(5000);
    wait_ms(50);
    off = (int16_t)vbus() - 5000;
    if (off > 300 || off < -300)
        off = 0;
    set_lv(LV_0V6, LV_3V3);
    wait_ms(T_CONT_ENTER);
    uint16_t v0 = vbus();
    *base = v0;
    for (uint8_t i = 0; i < 6; i++)     /* 3 个升压脉冲，应升约 600mV */
    {
        set_lv((i & 1) ? LV_0V6 : LV_3V3, LV_3V3);
        delay_us(T_PULSE_US);
    }
    wait_ms(150);
    uint16_t v1 = vbus();
    *dv = v1 > v0 ? v1 - v0 : 0;
    if (*dv > 300)
        f |= QC_3 | QC_9V | QC_12V;
    else
    {
        set_fixed(9000);
        wait_ms(250);
        if (vbus() > 8000)
            f |= QC_9V;
        set_fixed(12000);
        wait_ms(250);
        if (vbus() > 11000)
            f |= QC_12V;
    }
    set_fixed(5000);
    return f;
}

void qc_set(uint16_t mv, bool continuous)
{
    if (!continuous)
    {
        set_fixed(mv);
        steps = 0;
        phase = 3;
        settle = T_FIXED_MS;
        ts = millis();
        return;
    }
    /* 标称值 = 目标 − 偏差，按 200mV 取整，使实际输出离目标最近（≤ ±100mV） */
    int32_t n = ((int32_t)mv - off + 100) / 200 * 200;
    mv = n < QC3_MIN_MV ? QC3_MIN_MV : (uint16_t)n;
    target = mv;
    if (!cont)
    {
        /* 进连续模式：过了充电器去抖再按实测 VBUS 定起点（phase 4） */
        set_lv(LV_0V6, LV_3V3);
        cont = true;
        phase = 4;
        ts = millis();
        return;
    }
    start_steps();
}

static void start_steps(void)
{
    uint16_t mv = target;
    steps = (int8_t)(((int16_t)mv - (int16_t)cur_mv) / 200);
    cur_mv = mv;
    phase = steps ? 2 : 3;
    settle = steps ? T_CONT_SETTLE : 0;
    ts = millis();
    tu = micros();
}

void qc_process(void)
{
    uint32_t now = millis();
    switch (phase)
    {
    case 1:     /* 脉冲有效 → 回到连续模式电平（脉宽用 µs 计时：millis 的 1ms 粒度会让脉宽在 0~1ms 间随机） */
        if (micros() - tu >= T_PULSE_US)
        {
            set_lv(LV_0V6, LV_3V3);
            tu = micros();
            phase = 2;
        }
        break;
    case 2:
        if (micros() - tu >= T_PULSE_US)
        {
            if (!steps)
            {
                phase = 3;
                ts = now;
            }
            else
            {
                if (steps > 0)
                {
                    set_lv(LV_3V3, LV_3V3);
                    steps--;
                }
                else
                {
                    set_lv(LV_0V6, LV_0V6);
                    steps++;
                }
                tu = micros();
                phase = 1;
            }
        }
        break;
    case 3:
        if (now - ts >= settle)
            phase = 0;
        break;
    case 4:
        if (now - ts >= T_CONT_ENTER)
        {
            cur_mv = (uint16_t)(((int32_t)vbus() - off + 100) / 200 * 200);
            start_steps();
        }
        break;
    default:
        break;
    }
}

bool qc_busy(void)
{
    return phase != 0;
}
