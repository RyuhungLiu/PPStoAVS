#pragma once

#include <stdint.h>

void analog_init(void);
void analog_calibrate_current_zero(void);   /* MOS 关断（零电流）时调用 */
uint16_t analog_vbus_mv(void);              /* 前端 VBUS */
uint16_t analog_current_ma(void);           /* 输出电流（R4 低侧） */
