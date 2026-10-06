/*
 * QC2.0 / QC3.0 前端（见 qc.c）。fe_ufcs.c 把它当作另一种非 PD 前端：合成 PD 能力，协议桥不知道前端是哪种协议。
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define QC_OK       0x01        /* 握手成功（至少 5V） */
#define QC_9V       0x02
#define QC_12V      0x04
#define QC_3        0x08        /* 连续模式（±200mV） */

void qc_set_yield(void (*fn)(void));
uint8_t qc_connect(uint16_t *dv, uint16_t *base);  /* 阻塞约 2.5s；base = QC3 测试前（已进连续模式）的 VBUS，dv = 3 个升压脉冲后升高的 mV；返回 QC_* 位，0 = 不是 QC 充电器（D± 已放开） */
void qc_set(uint16_t mv, bool continuous);  /* 开始切换；continuous = QC3 连续模式（按 200mV 取整） */
void qc_process(void);
bool qc_busy(void);
