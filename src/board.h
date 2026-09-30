/*
 * PPStoAVS 板级定义（CH32M030K9U7，PCB v0.1 2026-09-19）
 *
 * 前端（公头 USB3，接 PPS 充电器）：PA3/CC4R → USBPD1，Sink，内置 Rd
 * 后端（母座 USB1，接设备）       ：PA0/CC1、PA1/CC2 → USBPD0，Source，Rp 电流源
 * 电源路径：Q3/Q1 背靠背 NMOS，栅极由 PB15/HVOD3 控制，R3 150K 上拉到 HVCP
 */
#pragma once

#include "ch32m030.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* ---------------- 电源开关 ---------------- */
/* PB15 通用推挽输出：OUTDR=1 → 引脚拉低 → MOS 关断；OUTDR=0 → 高阻 → R3 上拉 → MOS 导通 */
#define GATE_GPIO_PORT          GPIOB
#define GATE_GPIO_PIN           GPIO_Pin_15

/* 电荷泵：TIM2_RM=10，PC5=TIM2_CH2(PWMOUT)，PB14=CH2N，PB12=CH1N（DS2 §4） */
#define HVCP_PWM_PERIOD_TICKS   96      /* 48MHz / 96 = 500kHz，周期 2us（手册 1~5us） */
#define HVCP_DEADTIME_DT2       4       /* 5T ≈ 104ns（手册约 100ns） */
#define HVCP_STARTUP_MS         10      /* C3 100nF 充电时间 */

/* ---------------- 模拟量 ---------------- */
#define VBUS_ADC_PORT           GPIOA
#define VBUS_ADC_PIN            GPIO_Pin_13
#define VBUS_ADC_CHANNEL        ADC_Channel_18      /* PA13/ADC_IN18，R12 100K / R13 10K */
#define VBUS_DIVIDER_NUM        11                  /* (100K + 10K) / 10K */
#define ADC_VREF_MV             3300                /* VDD33 标称值，实测偏差可在此校正 */

#define ISP_ADC_CHANNEL         ADC_Channel_10      /* OPA4(ISP2) 输出在内部接 ADC_IN10 */
#define FE_CC_ADC_CHANNEL       ADC_Channel_16      /* PA3/ADC_IN16：前端 CC 电压（判断 SinkTxOK/NG） */
#define ISP_GPIO_PORT           GPIOA
#define ISP_GPIO_PINS           (GPIO_Pin_10 | GPIO_Pin_11)
#define ISP_GAIN                55
#define ISP_RSENSE_MOHM         5                   /* R4 5mΩ，低侧，ISP 接 GND_1 */

/* ---------------- CC 引脚 ---------------- */
#define BE_CC_PORT              GPIOA
#define BE_CC_PINS              (GPIO_Pin_0 | GPIO_Pin_1)
#define FE_CC_PORT              GPIOA
#define FE_CC_PIN               GPIO_Pin_3          /* 同时是 SWIO（DIO 测试点） */
#define FE_RA_PIN               GPIO_Pin_2          /* PA2/CC3：经 1kΩ 接公头 B5（VCONN），拉低 = Ra（硬件改版后） */

/* 上电后保留给调试器的窗口，之后关闭 SDI，把 PA3 交给 USBPD1 */
#define BOOT_DEBUG_WINDOW_MS    300

/* ---------------- 产品能力与保护 ---------------- */
/* 最高电压、电流上限、OVP/UVP 容差、OCP 由上位机设置（cfg.c，默认 20V / 3A / ±5% / 3.5A 50ms） */
#define FW_VERSION              0x0090  /* 0xMMmp：v0.9.0 */
#define VBUS_STABLE_SAMPLES     3       /* 连续 3 次采样在窗口内即判稳定 */
#define VBUS_SAMPLE_INTERVAL_MS 2
#define PPS_KEEPALIVE_MS        5000    /* 规范 tPPSRequest ≤ 10s */
