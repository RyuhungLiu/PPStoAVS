/*
 * 前端 D+/D-（PB0/PB1，USBFS）自定义 HID：64 字节 IN/OUT 报告，供上位机（WebHID）使用
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
