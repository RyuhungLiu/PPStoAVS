/*
 * 协议桥：前端充电器能力 → 后端 Source_Capabilities；后端请求 → 前端请求；电压确认与保护
 */
#pragma once

#include "fe_sink.h"
#include "pd_defs.h"

typedef enum
{
    BRIDGE_PENDING,
    BRIDGE_OK,
    BRIDGE_FAIL,
} bridge_result_t;

/* 前端回调 */
fe_target_t bridge_on_front_caps(void);     /* 收到充电器能力（含 DPS 重新广播），返回应立即请求的目标 */
void bridge_on_front_reset(void);           /* 前端 Hard Reset / Soft Reset */

/* 后端调用 */
bool bridge_front_ready_for_vsafe5v(void);  /* 前端处于 5V 且空闲，可以打开 MOS */
uint8_t bridge_back_caps(uint32_t *pdos);   /* 生成后端 Source_Capabilities，返回个数 */
bool bridge_eval_back_request(uint32_t rdo);
void bridge_start_transition(void);
bridge_result_t bridge_poll_transition(void);
void bridge_on_back_contract(void);         /* 已向设备发出 PS_RDY */
void bridge_on_back_reset(void);            /* 后端断开或 Hard Reset：停止监测，前端回 5V */

bool bridge_take_back_caps_dirty(void);     /* DPS 等原因需要重新广播 */
bool bridge_take_back_hard_reset(void);     /* 当前合约失效或保护动作 */

void bridge_process(void);                  /* 前端回 5V 请求、过/欠压、过流监测 */
