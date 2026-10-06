/*
 * SCP（及 AFC / FCP）前端探测固件（-DHV_PROBE）：上电在协议桥之前阻塞执行一次，结果写入事件记录（EV_HV_STEP / EV_HV_EDGES / EV_HV_BYTES）。
 * 共用 HiSilicon 的 D− 单线物理层（BC1.2/DCP 后 D+ 保持 0.6V，D− 双向，只由设备端发起）：
 *   UI = 160µs；主机 Ping = D− 高 16 UI；从机 Ping 高 10~20 UI（实测 16 UI）；
 *   字节：同步 = 1/4 UI 高-低-高（第三段可与数据位 1 相连）→ 8 位数据（MSB 先，每位 1 UI，高 = 1）→ 奇校验位
 *   （发送照三星开源内核 drivers/afc/gpio_afc.c；从机回应格式由 v0.14 探测实测：Sping → 约 4 UI 低 → 字节… → Sping）
 *  AFC（已摸清，此版不跑，afc_set 保留）：送 V/I 字节，接受则回显；不支持则回自己的档位表（session 32：0x0B 0x49 0x79）
 *  1. qc_handshake：D+ 0.6V，D− 跟随（短接）→ 等充电器放开 D−
 *  2. SCP（寄存器协议，命令 SBRRD 0x0C / SBRWR 0x0B；寄存器见华为开源内核 direct_charger.h；CRC 未知）：
 *     10 种 CRC 方案（不带 / 常见 CRC-8）各读一次 SCP_ADP_TYPE(0x80)，有回应的记下解出的字节；
 *     用第一个有回应的方案读 0x81、0x90、0x92~0x95、FCP 0x00 / 0x21；
 *     再试低压直充：CTRL_BYTE0(0xA0) = 0x40 → VSSET(0xCA) = 5.5V → 0xA0 = 0xC0，量 VBUS → 0xA0 = 0x20 复位
 *  3. 保持 D+ 0.6V，返回 true：主循环只写记录
 * 注意：AFC 充电器会把 SBRRD 的第一个字节 0x0C 当成 AFC 请求而回档位表，看起来像“有回应”。
 * D− 发送用 UDM 缓冲 DAC（0 / 63 ≈ 3.25V），接收用 UDM 比较器（阈值 20/64 × 3.3V ≈ 1.03V）。后端开关保持关断。
 */
#ifdef HV_PROBE

#include "hv_probe.h"
#include "analog.h"
#include "board.h"
#include "evlog.h"
#include "hisi_phy.h"
#include "qc.h"
#include "timebase.h"

#define UI HP_UI

static uint32_t t0;
static uint16_t vbus_avg(void)
{
    uint32_t s = 0;
    for (uint8_t i = 0; i < 8; i++)
        s += analog_vbus_mv();
    return (uint16_t)(s / 8);
}

static void log_step(uint8_t code, uint8_t arg, uint16_t x)
{
    ev_hv_step_t e = {code, arg, x, vbus_avg(), (uint16_t)(millis() - t0)};
    evlog_add(EV_HV_STEP, &e, sizeof(e));
}

/* 立即封页落盘：v0.14 实测探测中途会重启（握手后几个事务），RAM 里没封页的记录会丢 */
static void checkpoint(void)
{
    evlog_panic_flush();
}

/* 等待期间顺便把记录写进 Flash（预擦除下一页）；VBUS 掉到 4V 以下时立即记一笔并落盘（充电器断电） */
static void wait_ms(uint32_t ms)
{
    static bool lost;
    uint32_t s = millis(), last = 0;
    while (millis() - s < ms)
    {
        if (millis() - last >= 50 && evlog_flush_step())
            last = millis();
        uint16_t v = analog_vbus_mv();
        if (!lost && v < 4000)
        {
            lost = true;
            log_step(HVS_VBUS, 9, v);
            checkpoint();
        }
    }
}

static void log_edges(uint8_t tag)
{
    uint8_t buf[3 + 48 * 2];
    for (uint8_t o = 0; o < hp_n_edges || o == 0; o += 48)
    {
        uint8_t k = hp_n_edges - o > 48 ? 48 : hp_n_edges - o;
        buf[0] = tag;
        buf[1] = (uint8_t)(hp_first_level ^ (o & 1)) | (uint8_t)(o << 1);     /* bit0 首段电平，bit1~ 起始序号 */
        buf[2] = k;
        for (uint8_t i = 0; i < k; i++)
        {
            buf[3 + i * 2] = hp_edges[o + i] & 0xFF;
            buf[4 + i * 2] = hp_edges[o + i] >> 8;
        }
        evlog_add(EV_HV_EDGES, buf, 3 + k * 2);
        if (!hp_n_edges)
            break;
    }
}

/* AFC 一轮：返回 0 成功，1~4 = 第几个 Sping 失败 */
/* AFC 一轮：返回 0 成功，1~4 = 第几个 Sping 失败；cap = 第二个 Mping 后改为记录从机回应波形（tag） */
static uint8_t afc_once(uint8_t data, int32_t sp[4], uint8_t cap)
{
    hp_mping();
    if ((sp[0] = hp_sping()) < 0)
        return 1;
    hp_send_byte(data);
    hp_mping();
    if (cap)
    {
        hp_capture(12);
        log_edges(cap);
    }
    else
    {
        if ((sp[1] = hp_sping()) < 0)
            return 2;
        delay_us(2000);         /* 从机数据 */
        if ((sp[2] = hp_sping()) < 0)
            return 3;
    }
    delay_us(200);
    hp_mping();
    if ((sp[3] = hp_sping()) < 0)
        return 4;
    return 0;
}

__attribute__((unused)) static bool afc_set(uint8_t data, uint8_t cap)
{
    uint8_t ok = 0;
    for (uint8_t r = 0; r < 3; r++)
    {
        int32_t sp[4] = {0, 0, 0, 0};
        uint8_t res = afc_once(data, sp, r == 0 ? cap : 0);
        hp_out(false);
        log_step(HVS_AFC, (uint8_t)(res | (r << 4)), (uint16_t)(sp[0] > 0 ? sp[0] : 0));
        ok += res == 0;
        wait_ms(38);
    }
    return ok > 0;
}

/* FCP 的 CRC 未知：逐个试常见 CRC-8（0 = 不带 CRC）；refl = 按位反转（LSB 先）算法 */
static const struct { uint8_t poly, init, refl; } crcs[] = {
    {0, 0, 0},       {0x07, 0x00, 0}, {0x07, 0xFF, 0}, {0x31, 0x00, 0}, {0x31, 0xFF, 0},
    {0x1D, 0xFF, 0}, {0x9B, 0x00, 0}, {0xD5, 0x00, 0}, {0x8C, 0x00, 1}, {0xE0, 0x00, 1},
};
#define N_CRC (sizeof(crcs) / sizeof(crcs[0]))

static uint8_t crc8(uint8_t v, const uint8_t *p, uint8_t n)
{
    uint8_t c = crcs[v].init, poly = crcs[v].poly;
    while (n--)
    {
        c ^= *p++;
        for (uint8_t i = 0; i < 8; i++)
            if (crcs[v].refl)
                c = (c & 1) ? (uint8_t)((c >> 1) ^ poly) : (uint8_t)(c >> 1);
            else
                c = (c & 0x80) ? (uint8_t)((c << 1) ^ poly) : (uint8_t)(c << 1);
    }
    return c;
}

static void log_bytes(uint8_t tag)
{
    uint8_t buf[3 + 8];
    uint8_t bad;
    uint8_t n = hp_decode(&buf[3], 8, &bad);
    buf[0] = tag;
    buf[1] = bad;
    buf[2] = n;
    evlog_add(EV_HV_BYTES, buf, 3 + n);
}

/* SCP / FCP 事务表（命令 SBRRD 0x0C / SBRWR 0x0B，寄存器见华为开源内核 direct_charger.h、hw_scp.h） */
static const struct { uint8_t cmd, addr; int16_t data; } xfers[] = {
    {0, 0, 0},
    {0x0C, 0x80, -1},       /* 1  SCP_ADP_TYPE */
    {0x0C, 0x81, -1},       /* 2  SCP_B_ADP_TYPE */
    {0x0C, 0x90, -1},       /* 3  SCP_MAX_POWER */
    {0x0C, 0x92, -1},       /* 4  SCP_MIN_VOUT */
    {0x0C, 0x93, -1},       /* 5  SCP_MAX_VOUT */
    {0x0C, 0x94, -1},       /* 6  SCP_MIN_IOUT */
    {0x0C, 0x95, -1},       /* 7  SCP_MAX_IOUT */
    {0x0C, 0x00, -1},       /* 8  FCP DVCTYPE */
    {0x0C, 0x21, -1},       /* 9  FCP DISCRETE_CAPABILITIES */
    {0x0B, 0xA0, 0x40},     /* 10 SCP_CTRL_BYTE0 = 输出模式 */
    {0x0B, 0xCA, 250},      /* 11 SCP_VSSET = 3000 + 250×10 = 5.5V */
    {0x0B, 0xA0, 0xC0},     /* 12 SCP_CTRL_BYTE0 = 输出模式 + 输出使能 */
    {0x0B, 0xA0, 0x20},     /* 13 SCP_CTRL_BYTE0 = 适配器复位 */
};

/* 一次事务：Mping → Sping → 命令/地址/(数据)/(CRC) → Mping → 记录从机回应 → Mping → Sping；
 * 返回记录到的跳变数（只有从机 Ping 时为 2）。wave = 记录完整波形（否则只记解出的字节） */
static uint8_t xfer(uint8_t t, uint8_t v, bool wave)
{
    uint8_t b[4] = {xfers[t].cmd, xfers[t].addr, (uint8_t)xfers[t].data, 0};
    uint8_t n = xfers[t].data >= 0 ? 3 : 2;
    if (v)
    {
        b[n] = crc8(v, b, n);
        n++;
    }
    hp_mping();
    int32_t s0 = hp_sping();
    hp_n_edges = 0;
    if (s0 >= 0)
    {
        for (uint8_t i = 0; i < n; i++)
            hp_send_byte(b[i]);
        hp_mping();
        hp_capture(15);
        hp_out(false);
        delay_us(200);
        hp_mping();
    }
    int32_t s1 = s0 >= 0 ? hp_sping() : 0;
    hp_out(false);
    uint8_t got = hp_n_edges, tag = (uint8_t)(t | (v << 4));
    log_step(HVS_FCP, tag, (uint16_t)(got | (s0 < 0 ? 0x100 : 0) | (s1 < 0 ? 0x200 : 0)));
    if (got > 2)
        log_bytes(tag);
    if (wave && s0 >= 0)
        log_edges(tag);
    checkpoint();
    wait_ms(40);
    return got;
}

bool hv_probe_boot(void)
{
    t0 = millis();
    uint16_t hs = qc_handshake();
    log_step(HVS_HANDSHAKE, hs != 0, hs);
    checkpoint();
    if (!hs)
    {
        wait_ms(2500);          /* 等记录封页写入 Flash，再交给 USB HID */
        return false;
    }
    hp_out(false);
    wait_ms(20);

    /* SCP：10 种 CRC 方案都读一次 SCP_ADP_TYPE(0x80)，有回应的都记下解出的字节；第一个有回应的方案用于其余读写 */
    int8_t good = -1;
    for (uint8_t c = 0; c < N_CRC; c++)
        if (xfer(1, c, c == 0) > 2 && good < 0)
            good = (int8_t)c;
    log_step(HVS_FCP_CRC, (uint8_t)good, 0);
    if (good >= 0)
    {
        for (uint8_t t = 2; t <= 9; t++)
            xfer(t, (uint8_t)good, true);
        /* 低压直充：输出模式 → 5.5V → 输出使能，量 VBUS；再复位适配器 */
        xfer(10, (uint8_t)good, true);
        xfer(11, (uint8_t)good, true);
        xfer(12, (uint8_t)good, true);
        wait_ms(300);
        log_step(HVS_VBUS, 3, vbus_avg());
        xfer(13, (uint8_t)good, true);
        wait_ms(500);
        log_step(HVS_VBUS, 5, vbus_avg());
    }

    hp_out(false);
    log_step(HVS_DONE, 0, 0);
    return true;
}

void hv_probe_idle(void)
{
    static uint32_t last;
    if (millis() - last >= 50 && evlog_flush_step())
        last = millis();
}

#endif
