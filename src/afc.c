/*
 * 三星 AFC 前端（Sink）。物理层见 hisi_phy.c；协议由探测固件实测（session 29 / 32）：
 *  - 一字节 V/I 请求：高 4 位 = 电压 − 5V（1V 一级），低 4 位 = 电流 0.75A + n × 0.15A
 *  - 事务：Mping → Sping → 请求字节 → Mping → Sping + 从机字节 + Sping → Mping → Sping
 *  - 充电器支持该 V/I 时回显同一字节并切换电压（三星代码连做 3 轮，间隔 38ms）；
 *    不支持时回自己的 V/I 表（实测 0x0B 0x49 0x79 = 5V 2.4A / 9V 2.1A / 12V 2.1A）
 * 能力查询：发一个 5V 档、电流少见的字节（不会改变电压），取回 V/I 表。
 * 切换：每轮约 22ms（阻塞，发送字节时关中断约 1.8ms），轮间 38ms 非阻塞；3 轮回显后等 VBUS 到位（≤ 800ms）。
 */
#include "afc.h"
#include "analog.h"
#include "hisi_phy.h"
#include "timebase.h"

#define T_ROUND_GAP_MS  38
#define N_ROUNDS_OK     3
#define N_ROUNDS_MAX    6
#define T_SETTLE_MS     800

static uint8_t req, rounds, oks, phase;     /* phase：0 空闲，1 轮次进行中，2 等 VBUS */
static bool result;
static uint32_t ts;

uint16_t afc_mv(uint8_t vi)
{
    return 5000 + (vi >> 4) * 1000;
}

uint16_t afc_ma(uint8_t vi)
{
    return 750 + (vi & 15) * 150;
}

/* 一轮：返回从机字节数（不含 Ping），解出的字节放 out */
static uint8_t round_once(uint8_t vi, uint8_t *out, uint8_t max)
{
    int32_t s0, s1;
    hp_xfer(&vi, 1, 10, &s0, &s1);
    if (s0 < 0)
        return 0;
    uint8_t bad;
    uint8_t n = hp_decode(out, max, &bad);
    return bad ? 0 : n;
}

uint8_t afc_query(uint8_t *list, uint8_t max)
{
    static const uint8_t probe[] = {0x0C, 0x0D, 0x0E, 0x0F, 0x01};
    hp_out(false);
    delay_ms(20);
    for (uint8_t i = 0; i < sizeof(probe); i++)
    {
        uint8_t n = round_once(probe[i], list, max);
        if (n && !(n == 1 && list[0] == probe[i]))
            return n;                       /* 不是回显：这就是 V/I 表 */
        delay_ms(T_ROUND_GAP_MS);
    }
    return 0;
}

void afc_set(uint8_t vi)
{
    req = vi;
    rounds = oks = 0;
    phase = 1;
    ts = millis() - T_ROUND_GAP_MS;
}

void afc_process(void)
{
    uint32_t now = millis();
    if (phase == 1 && now - ts >= T_ROUND_GAP_MS)
    {
        uint8_t b[4];
        uint8_t n = round_once(req, b, sizeof(b));
        rounds++;
        oks += n == 1 && b[0] == req;
        ts = millis();
        if (oks >= N_ROUNDS_OK || rounds >= N_ROUNDS_MAX)
        {
            if (!oks)
            {
                result = false;
                phase = 0;
            }
            else
                phase = 2;
        }
    }
    else if (phase == 2)
    {
        uint16_t want = afc_mv(req), v = analog_vbus_mv();
        if (v > want - want / 12 && v < want + want / 12)
        {
            result = true;
            phase = 0;
        }
        else if (now - ts > T_SETTLE_MS)
        {
            result = false;
            phase = 0;
        }
    }
}

bool afc_busy(void)
{
    return phase != 0;
}

bool afc_ok(void)
{
    return result;
}
