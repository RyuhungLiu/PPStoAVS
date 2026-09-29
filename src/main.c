/*
 * PPStoAVS 固件主程序
 *
 * 上电顺序：
 *  1. PB15 拉低，保证 MOS 关断（DS2：PB15 复位后为高阻，此时栅极会被 R3 上拉）
 *  2. 时基、ADC/OPA4，MOS 关断状态下校准电流零点
 *  3. 启动 HVCP 电荷泵
 *  4. 初始化后端 USBPD0（Source，开始检测设备）
 *  5. 读取设置、扫描记录区并预擦除（PD 尚未启动，Flash 停顿无影响）
 *  6. 前端 D+/D- USB HID（上位机）；探测固件（-DUFCS_PROBE）先尝试 UFCS，成功则前端由 UFCS 独占
 *  7. 保留调试窗口后关闭 SDI，初始化前端 USBPD1（Sink，PA3/CC4R）
 *  8. 主循环：前端 → 协议桥 → 后端 → 上位机 → Flash 写入调度，喂狗
 */
#include "analog.h"
#include "be_source.h"
#include "board.h"
#include "bridge.h"
#include "cfg.h"
#include "evlog.h"
#include "fe_sink.h"
#include "fe_ufcs.h"
#include "host.h"
#include "pd_phy.h"
#include "power_sw.h"
#include "store.h"
#include "timebase.h"
#include "ufcs.h"
#include "usb_hid.h"

static void watchdog_init(void)
{
    /* 48MHz / 4096 / 8 ≈ 1.46kHz，计数 0x7F→0x3F 约 43ms 超时 */
    RCC_PB1PeriphClockCmd(RCC_PB1Periph_WWDG, ENABLE);
    WWDG_SetPrescaler(WWDG_Prescaler_8);
    WWDG_SetWindowValue(0x7F);
    WWDG_Enable(0x7F);
}

static void watchdog_feed(void)
{
    WWDG_SetCounter(0x7F);
}

int main(void)
{
    power_sw_early_off();

    SystemCoreClockUpdate();
    timebase_init();

    /* 复位原因：RCC_RSTSCKR[31:24]，读出后清除 */
    uint8_t reset_flags = RCC->RSTSCKR >> 24;
    RCC->RSTSCKR |= RCC_RMVF;

    cfg_init();
    fe_ra_apply();      /* 虚拟 E-Marker：尽早呈现 Ra */
    evlog_init();
    ev_boot_t boot = {FW_VERSION, reset_flags, *cfg()};
    evlog_add(EV_BOOT, &boot, sizeof(boot));

    analog_init();
    analog_calibrate_current_zero();

    power_sw_hvcp_start();

    pd_phy_hw_init();
    be_init();
    /* 前端 D+/D− 与 USB HID 共用：先看 UFCS（设置开启，或探测固件），握手成功则前端走 UFCS，不启动 HID 和 PD 前端 */
#ifdef UFCS_PROBE
    const bool ufcs_up = ufcs_probe_boot();     /* 阻塞：电平扫描 → 握手 → Ping（此时看门狗尚未启动） */
#else
    const bool ufcs_up = (cfg()->flags2 & CFG2_UFCS) && feu_start();
#endif
    if (!ufcs_up)
        usb_hid_init();

    watchdog_init();
    ufcs_set_yield(watchdog_feed);

#ifdef UFCS_PROBE
    if (ufcs_up)
    {
        while (1)
        {
            ufcs_process();
            store_process();
            watchdog_feed();
        }
    }
#endif

#ifndef FE_DIAG     /* 诊断构建：不留调试窗口，上电立即接管前端（重烧用断电擦除） */
    /* 虚拟 E-Marker：充电器在首次广播前查询线材，同样立即接管前端（重烧用断电擦除） */
    while (!ufcs_up && millis() < BOOT_DEBUG_WINDOW_MS && !(cfg()->flags & CFG_FE_EMARKER))
    {
        be_process();
        watchdog_feed();
    }
#endif
    if (!ufcs_up)
        fe_init();

    while (1)
    {
        fe_process();
        bridge_process();
        be_process();
        if (!ufcs_up)
            host_process();
        store_process();
        watchdog_feed();
    }
}
