/*
 * 上位机协议（USB HID，64 字节报告，小端）
 *
 * 请求：cmd u8 | tag u8 | payload[62]
 * 应答：cmd|0x80 u8 | tag u8 | status u8 | payload[61]
 *
 * 0x01 INFO         → proto u8, fw u16, session u16, uptime_ms u32, dropped u16, flags u8, log_pages u8
 *                      flags：bit0 设置保存中，bit1 记录清除中
 * 0x02 STATUS part  → part 0：fe_state u8, be_state u8, flags u8, vbus_mv u16, ibus_ma u16,
 *                              fe_rdo u32, be_rdo u32, be_mv u16, n u8, 充电器 PDO[n]
 *                              flags：bit0 前端非 PD，bit1 后端已插入，bit2 MOS 导通，bit3 后端合约，bit4 前端合约，
 *                                     bit6..5 后端 AVS 二次握手（0 不适用，1 等待查询，2 设备不支持，3 已给 AVS）
 *                      part 1：n u8, 后端能力 PDO[n], 对应充电器位置 src u8[n]（0 = 无）
 *                      part 2：n u8, 前端 EPR 能力 PDO[n]（8 号起）
 *                      part 3：PD 信息透传（设备电量、双方身份，格式见 pdinfo_status）
 *                      part 4：后端线材 E-Marker（协议 v10 起，每次连接都读，不看 Ra）：status u8（be_cable_status_t）, flags u8, n u8, vdo u32[n]
 * 0x10 CFG_GET      → cfg_t（32 字节，协议 v8 起；v6 为 24 字节，v5 为 20 字节）, save_pending u8
 * 0x11 CFG_SET cfg  → 校验失败返回 BAD_ARG；成功立即生效并排队保存
 * 0x12 CFG_DEFAULT  → 恢复默认设置
 * 0x20 LOG_SESSIONS → current u16, ram_mask u8（bit0 当前页，bit1/2 待写页），n u8, {session u16, first_idx u8, pages u8}[n]
 * 0x21 LOG_READ idx off → idx u8, off u8, len u8, data[len ≤ 56]（idx 0~127 Flash 页，0xFF/0xFE/0xFD RAM 页）
 * 0x22 LOG_CLEAR    → 清除全部记录
 * 0x30 SYS_INFO    → 在线升级信息（协议 v9 起，格式见 iap.h）
 * 0x31 ENTER_BL "BL" → 复位进入 Bootloader（后端输出会中断）；之后用 iap.h 里的 0x40 系列命令刷写
 */
#pragma once

#define HOST_PROTO_VERSION  10

void host_process(void);
