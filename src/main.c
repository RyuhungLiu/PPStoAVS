/*
 * PPStoAVS 固件主程序
 *
 * 上电顺序：
 *  1. PB15 拉低，保证 MOS 关断（DS2：PB15 复位后为高阻，此时栅极会被 R3 上拉）
 *  2. 时基、ADC/OPA4，MOS 关断状态下校准电流零点
 *  3. 启动 HVCP 电荷泵
 *  4. 初始化后端 USBPD0（Source，开始检测设备）
 *  5. 读取设置、扫描记录区并预擦除（PD 尚未启动，Flash 停顿无影响）
 *  6. 前端 D+/D- USB HID（上位机）
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
#include "host.h"
#include "pd_phy.h"
#include "power_sw.h"
#include "store.h"
#include "timebase.h"
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
    usb_hid_init();

    watchdog_init();

#ifndef FE_DIAG     /* 诊断构建：不留调试窗口，上电立即接管前端（重烧用断电擦除） */
    /* 虚拟 E-Marker：充电器在首次广播前查询线材，同样立即接管前端（重烧用断电擦除） */
    while (millis() < BOOT_DEBUG_WINDOW_MS && !(cfg()->flags & CFG_FE_EMARKER))
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
        host_process();
        store_process();
        watchdog_feed();
    }
}
