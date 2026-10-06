/*
 * 三星 AFC 前端（见 afc.c）。调用前须已完成 qc_handshake（D+ 保持 0.6V）。
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

uint16_t afc_mv(uint8_t vi);            /* V/I 字节 → 电压 mV */
uint16_t afc_ma(uint8_t vi);            /* V/I 字节 → 电流 mA */
uint8_t afc_query(uint8_t *list, uint8_t max);  /* 阻塞：取充电器的 V/I 表，返回字节数，0 = 不是 AFC 充电器 */
void afc_set(uint8_t vi);               /* 开始切换（3 轮回显 + 等 VBUS） */
void afc_process(void);
bool afc_busy(void);
bool afc_ok(void);                      /* 最近一次切换结果 */
