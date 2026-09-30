/*
 * 片内 Flash 页操作（快速模式，128 字节/页）与分区布局
 *
 *   0x0000_0000 ~ 0x0000_17FF  Bootloader（6 KB，bl/，复位后先运行，见 iap.h）
 *   0x0000_1800 ~ 0x0000_BDFF  程序（链接脚本 FLASH ORIGIN = 0x1800，LENGTH = 0xA600，由 Makefile 生成；1 KB 对齐，mtvec 需要）
 *   0x0000_BE00 ~ 0x0000_BE7F  进入 Bootloader 的标志页（程序写入，Bootloader 读出后擦除）
 *   0x0000_BE80 ~ 0x0000_BEFF  程序头页（Bootloader 校验通过后最后写入；擦除即视为无效程序）
 *   0x0800_BF00 ~ 0x0800_BFFF  配置 2 页（A/B 交替）
 *   0x0800_C000 ~ 0x0800_FFFF  透传记录 128 页环形
 *
 * 擦除/编程各约 4.5ms（DS 表 3-13），期间 CPU 取指停顿，由 store 模块挑 PD 空闲时调用。
 * 程序只允许写配置/记录区与标志页；Bootloader（-DBOOTLOADER）只允许写程序区、标志页与程序头页。
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define FLASH_PHYS_BASE     0x08000000u
#define FLASH_PAGE_SIZE     128u

#define BL_SIZE             0x1800u
#define APP_BASE            0x1800u
#define BL_FLAG_OFFSET      0xBE00u
#define APP_HDR_OFFSET      0xBE80u
#define APP_MAX_SIZE        (BL_FLAG_OFFSET - APP_BASE)
#define APP_FLASH_SIZE      0xBF00u     /* 程序区 + 标志页 + 程序头页的结束偏移 */

#define APP_ADDR            (FLASH_PHYS_BASE + APP_BASE)
#define BL_FLAG_ADDR        (FLASH_PHYS_BASE + BL_FLAG_OFFSET)
#define APP_HDR_ADDR        (FLASH_PHYS_BASE + APP_HDR_OFFSET)
#define CFG_FLASH_ADDR      (FLASH_PHYS_BASE + 0xBF00u)
#define CFG_FLASH_PAGES     2u
#define LOG_FLASH_ADDR      (FLASH_PHYS_BASE + 0xC000u)
#define LOG_FLASH_PAGES     128u

bool flash_page_erase(uint32_t addr);
bool flash_page_program(uint32_t addr, const uint32_t *data);   /* 32 个字，调用前该页须已擦除 */

uint32_t crc32_calc(const void *data, uint32_t len);
