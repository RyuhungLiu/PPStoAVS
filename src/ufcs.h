/*
 * UFCS 充电设备（Sink）协议引擎与探测流程（T/TAF 083-2022）
 *
 * 探测固件（-DUFCS_PROBE）：上电先做 DCP 判别与握手，成功后 Ping → 读能力 → 逐档请求 → 周期读 Source_Info，
 * 每一步写入记录（EV_UFCS_PKT / EV_UFCS_STEP），之后接 PC 用上位机的「记录」页查看。
 * MOS 始终保持关断，后端不供电。
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "ufcs_codec.h"

/* 阻塞：电平扫描 → 握手 → Ping。bridge = true 为转换器用法（不做探测扫升压，记录精简）。
 * 返回 true = 会话已建立（由 ufcs_process 继续） */
bool ufcs_connect(bool bridge);

/* 转换器用法：能力、请求、状态 */
const ufcs_mode_t *ufcs_modes(uint8_t *n);      /* 充电器的输出模式（已收到能力后有效） */
bool ufcs_take_caps(void);                       /* 收到新的能力（读后清除） */
bool ufcs_request(uint8_t mode, uint16_t mv, uint16_t ma);     /* 空闲时发起请求；忙返回 false */
uint8_t ufcs_req_status(void);                   /* 0 = 进行中/无，1 = 成功（Power_Ready），2 = 失败（拒绝或超时） */
bool ufcs_idle(void);                            /* 会话正常且没有请求在进行 */
bool ufcs_dead(void);                            /* 会话已结束（充电器复位、退出或失败） */
void ufcs_close(void);                           /* 释放 D+/D− */
void ufcs_set_yield(void (*fn)(void));

/* 阻塞：DCP 判别 → 握手 → Ping。返回 true = UFCS 会话已建立（由 ufcs_process 继续），此时不应启动 USB HID 和 PD 前端 */
bool ufcs_probe_boot(void);

void ufcs_process(void);
bool ufcs_session_active(void);     /* 已进入 UFCS 会话（之后前端由 UFCS 独占） */
bool ufcs_flash_safe(void);         /* 空闲且离下一次轮询还远，可以做 Flash 操作 */
