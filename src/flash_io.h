/*
 * 片内 Flash 页操作（快速模式，128 字节/页）与分区布局
 *
 *   0x0000_0000 ~ 0x0000_17FF  Bootloader（6 KB，bl/，复位后先运行，见 iap.h）
 * 布局 2（v0.14.0 起，FLASH_LAYOUT = 2；Bootloader 一并更新，旧板须用 WCH-LinkE 断电擦除后烧完整镜像）：
 *   0x0000_1800 ~ 0x0000_ECFF  程序 54.5 KB（链接脚本 FLASH ORIGIN = 0x1800，LENGTH = 0xD500，由 Makefile 生成；1 KB 对齐，mtvec 需要）
 *   0x0000_ED00 ~ 0x0000_ED7F  进入 Bootloader 的标志页（程序写入，Bootloader 读出后擦除）
 *   0x0000_ED80 ~ 0x0000_EDFF  程序头页（Bootloader 校验通过后最后写入；擦除即视为无效程序）
 *   0x0800_EE00 ~ 0x0800_EEFF  配置 2 页（A/B 交替）
 *   0x0800_EF00 ~ 0x0800_EFFF  第二组自订 PDO 2 页（A/B 交替）
 *   0x0800_F000 ~ 0x0800_FFFF  透传记录 32 页环形
 * 布局 0（v0.13.x 及以前）：程序 0x1800 ~ 0xBDFF（0xA600），标志页 0xBE00，程序头 0xBE80，配置 0xBF00，PDO2 0xC000，记录 0xC100 起 126 页
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
#define FLASH_LAYOUT        2           /* 写在 app_info.reserved，网页据此拒绝与 Bootloader 布局不符的映像 */
#define BL_FLAG_OFFSET      0xED00u
#define APP_HDR_OFFSET      0xED80u
#define APP_MAX_SIZE        (BL_FLAG_OFFSET - APP_BASE)
#define APP_FLASH_SIZE      0xEE00u     /* 程序区 + 标志页 + 程序头页的结束偏移 */

#define APP_ADDR            (FLASH_PHYS_BASE + APP_BASE)
#define BL_FLAG_ADDR        (FLASH_PHYS_BASE + BL_FLAG_OFFSET)
#define APP_HDR_ADDR        (FLASH_PHYS_BASE + APP_HDR_OFFSET)
#define CFG_FLASH_ADDR      (FLASH_PHYS_BASE + 0xEE00u)
#define CFG_FLASH_PAGES     2u
#define EXT2_FLASH_ADDR     (FLASH_PHYS_BASE + 0xEF00u)     /* 第二组自订 PDO（v0.13.0 起） */
#define EXT2_FLASH_PAGES    2u
#define LOG_FLASH_ADDR      (FLASH_PHYS_BASE + 0xF000u)
#define LOG_FLASH_PAGES     32u

bool flash_page_erase(uint32_t addr);
bool flash_page_program(uint32_t addr, const uint32_t *data);   /* 32 个字，调用前该页须已擦除 */

uint32_t crc32_calc(const void *data, uint32_t len);
