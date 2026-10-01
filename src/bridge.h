/*
 * 协议桥：前端充电器能力 → 后端 Source_Capabilities；后端请求 → 前端请求；电压确认与保护
 */
#pragma once

#include "fe_sink.h"
#include <stdbool.h>
#include "pd_defs.h"

typedef enum
{
    BRIDGE_PENDING,
    BRIDGE_OK,
    BRIDGE_FAIL,
} bridge_result_t;

/* 前端回调 */
/* 收到充电器能力（含 Soft Reset 后重新广播），返回应立即请求的目标；*for_bridge = 该请求属于进行中的转换 */
fe_target_t bridge_on_front_caps(bool *for_bridge);
void bridge_on_front_reset(void);           /* 前端 Hard Reset / Soft Reset */

/* 上位机修改设置后调用：重建后端能力，必要时让后端 Hard Reset */
void bridge_on_cfg_changed(void);

/* 后端电流上限（见 bridge.c）与线材 E-Marker 5A 通知 */
uint16_t bridge_back_max_ma(void);
void bridge_on_back_cable(bool five_amp);

/* 后端调用 */
bool bridge_front_ready_for_vsafe5v(void);  /* 前端处于 5V 且空闲，可以打开 MOS */
uint8_t bridge_back_caps(uint32_t *pdos);   /* 生成后端 Source_Capabilities，返回个数 */
uint8_t bridge_back_caps_peek(uint8_t *blob);   /* 上位机：PDO[n] 后接充电器位置 src[n]（5n 字节），返回 n */
bool bridge_back_contract(uint32_t *rdo, uint16_t *mv);
bool bridge_eval_back_request(uint32_t rdo);
void bridge_start_transition(void);
bridge_result_t bridge_poll_transition(void);
void bridge_on_back_contract(void);         /* 已向设备发出 PS_RDY */
void bridge_on_back_reset(void);            /* 后端断开或 Hard Reset：停止监测，前端回 5V */
void bridge_on_back_detach(void);           /* 设备拔出（Lab 二次握手状态清零） */

bool bridge_take_back_caps_dirty(void);     /* DPS 等原因需要重新广播 */

/* Lab：后端 AVS 二次握手 */
bool bridge_rear_query_wanted(void);        /* 需要向设备查询 Sink_Capabilities_Extended */
void bridge_on_rear_sink_modes(int16_t modes);  /* 设备的 Sink Modes；-1 = 不支持或未回复 */
uint8_t bridge_rear_avs_2nd_state(void);    /* 0 未启用/不适用，1 等待查询，2 设备不支持 AVS，3 已改为广播 AVS */
bool bridge_take_back_hard_reset(void);     /* 当前合约失效或保护动作 */

bool bridge_flash_safe(void);               /* 没有进行中的转换，可以做 Flash 操作 */

void bridge_process(void);                  /* 前端回 5V 请求、过/欠压、过流监测 */
