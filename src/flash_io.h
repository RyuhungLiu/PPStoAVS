/*
 * 片内 Flash 页操作（快速模式，128 字节/页）与数据区布局
 *
 *   0x0000_0000 ~ 0x0000_BEFF  程序（链接脚本 FLASH LENGTH = 0xBF00，由 Makefile 生成）
 *   0x0800_BF00 ~ 0x0800_BFFF  配置 2 页（A/B 交替）
 *   0x0800_C000 ~ 0x0800_FFFF  透传记录 128 页环形
 *
 * 擦除/编程各约 4.5ms（DS 表 3-13），期间 CPU 取指停顿，由 store 模块挑 PD 空闲时调用。
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define FLASH_PHYS_BASE     0x08000000u
#define FLASH_PAGE_SIZE     128u
#define APP_FLASH_SIZE      0xBF00u
#define CFG_FLASH_ADDR      (FLASH_PHYS_BASE + 0xBF00u)
#define CFG_FLASH_PAGES     2u
#define LOG_FLASH_ADDR      (FLASH_PHYS_BASE + 0xC000u)
#define LOG_FLASH_PAGES     128u

bool flash_page_erase(uint32_t addr);
bool flash_page_program(uint32_t addr, const uint32_t *data);   /* 32 个字，调用前该页须已擦除 */

uint32_t crc32_calc(const void *data, uint32_t len);
