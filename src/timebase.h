#pragma once

#include <stdint.h>

void timebase_init(void);
uint32_t millis(void);
uint32_t micros(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
