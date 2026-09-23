#pragma once

#include <stdbool.h>

void power_sw_early_off(void);  /* main 第一件事：PB15 拉低，保证 MOS 关断 */
void power_sw_hvcp_start(void); /* 启动 2 级电荷泵（必须在 early_off 之后） */
void power_sw_set(bool on);
bool power_sw_is_on(void);
