/*
 * QC2.0 / QC3.0 前端诱骗探测（-DQC_PROBE，见 qc_probe.c）
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

bool qc_probe_boot(void);       /* 阻塞；false = 不是 DCP（照常启动 HID），true = 已探测完、保持 5V */
void qc_probe_idle(void);       /* 探测后主循环：只写记录 */
