/*
 * PPStoAVS 固件主程序
 *
 * 上电顺序：
 *  1. PB15 拉低，保证 MOS 关断（DS2：PB15 复位后为高阻，此时栅极会被 R3 上拉）
 *  2. 时基、ADC/OPA4，MOS 关断状态下校准电流零点
 *  3. 启动 HVCP 电荷泵
 *  4. 初始化后端 USBPD0（Source，开始检测设备）
 *  5. 保留调试窗口后关闭 SDI，初始化前端 USBPD1（Sink，PA3/CC4R）
 *  6. 主循环：前端 → 协议桥 → 后端，喂狗
 */
#include "analog.h"
#include "be_source.h"
#include "board.h"
#include "bridge.h"
#include "fe_sink.h"
#include "pd_phy.h"
#include "power_sw.h"
#include "timebase.h"

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

    analog_init();
    analog_calibrate_current_zero();

    power_sw_hvcp_start();

    pd_phy_hw_init();
    be_init();

    watchdog_init();

#ifndef FE_DIAG     /* 诊断构建：不留调试窗口，上电立即接管前端（重烧用断电擦除） */
    while (millis() < BOOT_DEBUG_WINDOW_MS)
    {
        be_process();
        watchdog_feed();
    }
#endif
    fe_init();

    while (1)
    {
        fe_process();
        bridge_process();
        be_process();
        watchdog_feed();
    }
}
