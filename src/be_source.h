/*
 * 后端 Source 策略引擎（USBPD0，PA0/CC1、PA1/CC2）
 * 按 PD R3.2 Source 流程实现
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

void be_init(void);
void be_process(void);
bool be_flash_safe(void);
uint8_t be_state_code(void);
bool be_attached(void);
