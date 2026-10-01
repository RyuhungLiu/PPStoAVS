/*
 * 后端 Source 策略引擎（USBPD0，PA0/CC1、PA1/CC2）
 * 按 PD R3.2 Source 流程实现
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

void be_init(void);
void be_process(void);
bool be_flash_safe(void);
uint8_t be_state_code(void);
bool be_attached(void);

/* 后端线材 E-Marker */
#define BE_CABLE_VDOS   5       /* ID Header、Cert Stat、Product VDO、Cable VDO1、Cable VDO2（有源线材） */
typedef enum
{
    CABLE_NONE     = 0,         /* 后端没有设备 */
    CABLE_PENDING  = 1,         /* 正在读取（每次连接都向线材发 Discover Identity，不看 Ra） */
    CABLE_NO_REPLY = 2,         /* 线材不应答：没有 E-Marker，或没有 VCONN 供电（Ra 是否出现见 be_cable_ra） */
    CABLE_NAK      = 3,         /* 线材拒绝 Discover Identity */
    CABLE_OK       = 4,         /* 已读到身份 */
    CABLE_VIRTUAL  = 5,         /* 策略 CFG3_CABLE5A：不读线材，按 5A 处理并以虚拟 E-Marker 应答设备 */
} be_cable_status_t;

uint8_t be_cable_status(void);
uint8_t be_cable_vdos(uint32_t *out);   /* 返回 VDO 个数（≤ BE_CABLE_VDOS） */
bool be_cable_ra(void);                 /* 本次连接另一根 CC 脚出现 Ra（仅供显示；单 CC 焊盘没有另一根 CC） */
bool be_cable_5a(void);                 /* E-Marker 声明 5A */
void be_vconn_set(bool on);             /* 弱函数：硬件有 VCONN 开关时覆盖 */
bool be_vconn_available(void);
