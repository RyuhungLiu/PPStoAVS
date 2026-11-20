/* SPDX-License-Identifier: MPL-2.0 */
/*
 * Lab 固定诱骗测试（设计见上层目录「設計-固定誘騙模式.md」）
 *
 * 启用后：上电即锁死后端 MOS（power_sw_lock），后端不初始化；前端按 UFCS → QC → AFC → PD 的顺序（关闭的、
 * 目标电压对它不合法的跳过）把充电器固定在目标电压，只用来测试充电器能否被诱骗到该电压，不接负载。
 *  - 只请求精确目标，协商过程不超过目标（QC 不跑 qc_detect）
 *  - 运行中前端电压连续 3 次超出 OVP/UVP 窗口 → 回到 5V 并锁住，重新上电才再试
 *  - 每次状态变化记 EV_TRIG（强制记录）
 */
#pragma once

#include "fe_sink.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    TRIG_OFF,
    TRIG_NEGOTIATING,       /* 正在试 proto */
    TRIG_ACTIVE,            /* 已固定在目标 */
    TRIG_UNAVAILABLE,       /* 所有开启的协议都给不出目标，停在 5V */
    TRIG_LOST,              /* 协议连接中断（充电器复位、UFCS 会话丢失），等重新协商 */
    TRIG_FAULT,             /* 电压超出窗口或重试次数用完，已回 5V 并锁住 */
    TRIG_SKIPPED,           /* 仅记录用：proto 被跳过或失败，接着试下一个 */
} trig_state_t;

typedef enum
{
    TRR_NONE,
    TRR_OFF,                /* 该协议没开启 */
    TRR_MV,                 /* 目标电压对该协议不合法 */
    TRR_NO_RESP,            /* 充电器没有回应该协议（握手失败、非 PD、没有能力报文） */
    TRR_NO_LEVEL,           /* 充电器没有该电压 */
    TRR_NOT_REACHED,        /* 已请求，但电压没到目标窗口 */
    TRR_REJECT,             /* 请求被拒绝或超时 */
    TRR_LOST,               /* 协议连接中断 */
    TRR_WINDOW,             /* 运行中电压超出窗口 */
    TRR_RETRIES,            /* 中断重试次数用完 */
} trig_reason_t;

void trig_boot(void);               /* cfg_init 之后调用：启用则锁死后端 MOS、强制记录 */
bool trig_active(void);             /* 本次上电处于诱骗测试（启动时决定，运行中不变） */
uint8_t trig_run_dcp(void);         /* 阻塞（看门狗启动前）：依次试 UFCS、QC、AFC；返回占用前端 D± 的协议（TRIGP_*），0 = 都没成功 */
bool trig_pd_wanted(void);          /* trig_run_dcp 之后：是否还要初始化 PD 前端 */
void trig_process(void);            /* 主循环：PD 协商跟踪、电压窗口监控、故障回 5V */

/* 协议桥的钩子（UFCS / PD 前端收到能力、前端复位时） */
fe_target_t trig_on_caps(void);
void trig_on_front_reset(void);

/* 上位机状态页 5 */
uint8_t trig_status(uint8_t *buf);
