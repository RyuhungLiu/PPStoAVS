/*
 * D+/D-（PB0/PB1，USBFS；新板经 SEL 切到前端座或后端焊盘）自定义 HID：64 字节 IN/OUT 报告，供上位机（WebHID）使用
 * VID:PID = 1209:0001（pid.codes 测试 ID），Usage Page 0xFF00
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define USB_HID_REPORT_LEN  64

void usb_hid_init(void);
bool usb_hid_configured(void);
bool usb_hid_receive(uint8_t *buf);         /* 取出一条主机报告（64 字节）；没有则返回 false */
bool usb_hid_send(const uint8_t *buf);      /* 发送一条报告；上一条未发完返回 false */
bool usb_hid_tx_busy(void);                 /* 上一条报告还没被主机取走 */
/* usb_hid_poll_route 的结果 */
#define USB_ROUTE_NONE      0
#define USB_ROUTE_REAR      1               /* 前端超时没被配置，已切到后端 */
#define USB_ROUTE_NO_SEL    2               /* 前端超时没被配置，但板子没有 SEL（旧板），不切换 */
#define USB_ROUTE_REAR_CFG  3               /* 在后端第一次被主机配置 */

bool usb_hid_has_sel(void);                 /* 板子有 USB 切换器（PA12 上有 100K 上拉；第一次调用时检测） */
void usb_hid_route(bool rear);              /* 断开上拉，SEL 切到后端 / 前端，再重新初始化（等主机重新列举） */
bool usb_hid_on_rear(void);
uint8_t usb_hid_poll_route(void);           /* 主循环调用：前端 USB_FRONT_TIMEOUT_MS 内从未被主机配置 → 切到后端（只切一次）；返回 USB_ROUTE_* */
