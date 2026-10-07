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
uint16_t qc_handshake(void);    /* BC1.2→HVDCP 握手（QC/AFC/FCP 共用）：成功返回充电器放开 D− 所用 ms（D+ 保持 0.6V），0 = 失败（D± 已放开） */
uint8_t qc_detect(uint16_t *dv, uint16_t *base);   /* qc_handshake 成功后调用，阻塞约 0.5~1s；base = QC3 测试前（已进连续模式）的 VBUS，dv = 3 个升压脉冲后升高的 mV；返回 QC_* 位（QC_OK 以外都没有 = 充电器不认 QC 档位） */
void qc_release(void);          /* 放开 D+/D− */
void qc_lines_low(void);        /* D+/D− 都驱动到 0V（让充电器退出 HVDCP、回到 BC1.2 短接） */
void qc_set(uint16_t mv, bool continuous);  /* 开始切换；continuous = QC3 连续模式（按 200mV 取整） */
void qc_process(void);
bool qc_busy(void);
