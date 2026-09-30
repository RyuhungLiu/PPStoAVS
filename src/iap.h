/*
 * 在线升级（IAP）：Bootloader（BL）与程序（APP）共用的定义
 *
 * 复位后总是先运行 BL：
 *   - 标志页写有 IAP_FLAG_MAGIC（APP 收到上位机“进入 BL”后写入）→ 擦除标志并留在 BL 模式
 *   - 否则程序头有效且程序 CRC 正确 → 跳转到 APP
 *   - 否则（新芯片、升级中断电、程序损坏）留在 BL 模式，等上位机写入
 * BL 模式下前端 D+/D- 仍是与 APP 相同的 USB HID（同 VID/PID，网页可以自动重连），后端不输出，PD 不工作。
 *
 * BL 上位机协议（报告格式同 host.h：请求 cmd|tag|payload，应答 cmd|0x80|tag|status|payload）：
 *   0x30 SYS_INFO   APP 与 BL 都有 → mode u8（1 APP，2 BL）, bl_ver u16, iap_proto u8, app_valid u8, app_ver u16,
 *                                     app_size u32, app_base u32, app_max u32
 *   0x31 ENTER_BL   仅 APP，payload "BL" → 写标志页后复位进入 BL
 *   0x40 BEGIN      size u32, crc32 u32, version u16 → 使旧程序失效，开始接收
 *   0x41 DATA       offset u32, len u8（≤ 56）, data[len] → offset 必须等于已接收长度；满一页时擦除并编程
 *   0x42 END        写入最后一页，校验 CRC32，通过后写入程序头 → crc32 u32
 *   0x43 BOOT       复位启动 APP（程序无效返回 NO_APP）
 */
#pragma once

#include "flash_io.h"

#define IAP_PROTO           1
#define BL_VERSION          0x0100      /* 0xMMmm */

#define IAP_HDR_MAGIC       0x31505041u /* 'APP1' */
#define IAP_FLAG_MAGIC      0x314C4224u /* '$BL1' */

#define CMD_SYS_INFO        0x30
#define CMD_ENTER_BL        0x31
#define CMD_BL_BEGIN        0x40
#define CMD_BL_DATA         0x41
#define CMD_BL_END          0x42
#define CMD_BL_BOOT         0x43

#define IAP_ST_OK           0
#define IAP_ST_BAD_CMD      1
#define IAP_ST_BAD_ARG      2
#define IAP_ST_BAD_STATE    3       /* 还没 BEGIN、偏移不连续、长度不足 */
#define IAP_ST_FLASH        4       /* 擦除或编程失败 */
#define IAP_ST_CRC          5       /* 整体 CRC32 不符，程序头未写入 */
#define IAP_ST_NO_APP       6       /* 没有有效程序 */

#define IAP_DATA_MAX        56

typedef struct
{
    uint32_t magic;         /* IAP_HDR_MAGIC */
    uint32_t size;          /* 程序字节数（≤ APP_MAX_SIZE，4 的倍数） */
    uint32_t crc;           /* 程序 CRC32 */
    uint16_t version;       /* FW_VERSION */
    uint16_t reserved;
    uint32_t hdr_crc;       /* 前 16 字节的 CRC32 */
} iap_hdr_t;

const iap_hdr_t *iap_hdr(void);
bool iap_hdr_ok(const iap_hdr_t *h);            /* 魔数、大小范围、头 CRC */
bool iap_app_valid(void);                       /* 头有效且程序 CRC 正确（约 20 ms） */
void iap_hdr_page(uint32_t page[FLASH_PAGE_SIZE / 4], uint32_t size, uint32_t crc, uint16_t version);
bool iap_flag_set(void);

#ifndef BOOTLOADER
void iap_enter_bootloader(void);                /* APP：写标志页并复位，不返回 */
#endif
