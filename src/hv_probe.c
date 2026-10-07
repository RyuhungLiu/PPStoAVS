/*
 * SCP（及 AFC / FCP）前端探测固件（-DHV_PROBE）：上电在协议桥之前阻塞执行一次，结果写入事件记录（EV_HV_STEP / EV_HV_EDGES / EV_HV_BYTES）。
 * 共用 HiSilicon 的 D− 单线物理层（BC1.2/DCP 后 D+ 保持 0.6V，D− 双向，只由设备端发起）：
 *   UI = 160µs；主机 Ping = D− 高 16 UI；从机 Ping 高 10~20 UI（实测 16 UI）；
 *   字节：同步 = 1/4 UI 高-低-高（第三段可与数据位 1 相连）→ 8 位数据（MSB 先，每位 1 UI，高 = 1）→ 奇校验位
 *   （发送照三星开源内核 drivers/afc/gpio_afc.c；从机回应格式由 v0.14 探测实测：Sping → 约 4 UI 低 → 字节… → Sping）
 *  AFC（已摸清，此版不跑，afc_set 保留）：送 V/I 字节，接受则回显；不支持则回自己的档位表（session 32：0x0B 0x49 0x79）
 *  1. qc_handshake：D+ 0.6V，D− 跟随（短接）→ 等充电器放开 D−
 *  2. 扫描（不带 CRC，每种模式前重新握手；某帧之后充电器不再回 Ping 则记下该值为“锁死值”、重新握手后继续）：单字节 0x00~0xFF、[v, 0x80]、[v, 0x00]，只记“从机 Ping 之外还有回应”的值与解出的字节，
 *     每 64 个值记进度并落盘。v0.14.5 实测华为 SCP 充电器对 SBRRD 0x0C + 地址 0x80（10 种 CRC 方案）只回 Ping、没有数据：
 *     0x0C / 0x0B 是华为内核写进交换芯片指令寄存器的值，线上的指令编码与帧结构都还不知道，所以改为扫描
 *  3. 最后掷 AFC 单字节 0x0C（AFC 充电器回 V/I 表；华为 SCP 充电器收到后不再回 Ping，所以放最后）
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

/* 掷入一帧（不带 CRC），返回回应跳变数（只有从机 Ping 为 2）；s0 < 0 = 主机 Ping 后没有从机 Ping */
static uint8_t raw_xfer(const uint8_t *tx, uint8_t n, int32_t *s0)
{
    int32_t s1;
    uint8_t got = hp_xfer(tx, n, 12, s0, &s1);
    return *s0 < 0 ? 0 : got;
}

/* 命中：一条 EV_HV_BYTES，tag = 0x40 | 模式，内容 = 扫描值 + 从机字节 */
static void log_hit(uint8_t mode, uint8_t value)
{
    uint8_t buf[3 + 1 + 8];
    uint8_t bad;
    uint8_t n = hp_decode(&buf[4], 8, &bad);
    buf[0] = (uint8_t)(0x40 | mode);
    buf[1] = (uint8_t)(bad << 1);
    buf[2] = (uint8_t)(n + 1);
    buf[3] = value;
    evlog_add(EV_HV_BYTES, buf, 4 + n);
}

/* 重新握手：D± 拉到 0V（D+ < 0.325V 让充电器退出 HVDCP、回到 BC1.2 短接），再走一次 HVDCP 握手 */
static bool rehandshake(void)
{
    qc_lines_low();
    wait_ms(100);
    uint16_t hs = qc_handshake();
    hp_out(false);
    wait_ms(20);
    return hs != 0;
}

/* 扫描：mode 1 = 单字节 [v]，2 = [v, 0x80]，3 = [v, 0x00]；只记有回应（多于从机 Ping）的值，每 64 个值记进度并落盘。
 * v0.14.5 实测：某些帧之后充电器不再回 Ping（session 1：单字节 0x0C 之后就再也没有 Sping）→ 记下“上一帧的值”为锁死值，
 * 重新握手后重试当前值；锁死超过 40 次或重新握手失败则中止。返回命中数 */
static uint8_t scan(uint8_t mode)
{
    uint8_t hits = 0, locks = 0;
    bool abort = false;
    for (uint16_t v = 0; v < 256 && !abort; v++)
    {
        uint8_t tx[2] = {(uint8_t)v, mode == 2 ? 0x80 : 0x00};
        uint8_t n = mode == 1 ? 1 : 2;
        int32_t s0;
        uint8_t got = raw_xfer(tx, n, &s0);
        if (s0 < 0)
        {
            uint8_t lk[4] = {(uint8_t)(0x50 | mode), 0, 1, (uint8_t)(v - 1)};
            evlog_add(EV_HV_BYTES, lk, sizeof(lk));     /* 锁死：上一帧（v − 1；v = 0 时为模式切换前的帧） */
            if (++locks > 40 || !rehandshake())
                abort = true;
            else
            {
                got = raw_xfer(tx, n, &s0);
                if (s0 < 0)
                    abort = true;               /* 刚握手完就发这个值也锁死：本值记为锁死值后中止 */
            }
        }
        if (!abort && got > 2 && hits < 40)
        {
            hits++;
            log_hit(mode, (uint8_t)v);
        }
        if ((v & 63) == 63 || abort)
        {
            log_step(HVS_SCAN, mode, (uint16_t)(v | (hits & 0x7F) << 8 | (abort ? 0x8000 : 0)));
            checkpoint();
        }
        wait_ms(15);
    }
    return hits;
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

    /* 1. 扫描单字节、[v, 0x80]、[v, 0x00]（不带 CRC）；每种模式前重新握手，从干净的状态开始 */
    for (uint8_t m = 1; m <= 3; m++)
    {
        if (m > 1 && !rehandshake())
        {
            log_step(HVS_HANDSHAKE, 0, 0);
            break;
        }
        scan(m);
        wait_ms(100);
    }

    /* 2. 最后才试 AFC 单字节 0x0C（session 1：华为 SCP 充电器收到它之后不再回 Ping，放在扫描前会毁掉整个扫描） */
    if (rehandshake())
    {
        uint8_t q = 0x0C;
        int32_t s0;
        uint8_t got = raw_xfer(&q, 1, &s0);
        log_step(HVS_FCP, 0x0C, (uint16_t)(got | (s0 < 0 ? 0x100 : 0)));
        if (got > 2)
            log_hit(0, q);
        log_edges(0x0C);
        q = 0x08;               /* AFC 5V（不是 AFC 充电器则无作用） */
        raw_xfer(&q, 1, &s0);
        wait_ms(300);
        log_step(HVS_VBUS, 2, vbus_avg());
    }

    hp_out(false);
    log_step(HVS_DONE, 0, 0);
    checkpoint();
    return true;
}

void hv_probe_idle(void)
{
    static uint32_t last;
    if (millis() - last >= 50 && evlog_flush_step())
        last = millis();
}

#endif
