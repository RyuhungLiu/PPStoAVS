/*
 * AFC / FCP 前端探测（-DHV_PROBE，见 hv_probe.c）
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

bool hv_probe_boot(void);       /* 阻塞；false = 握手失败（照常启动 HID），true = 已探测完 */
void hv_probe_idle(void);       /* 探测后主循环：只写记录 */
