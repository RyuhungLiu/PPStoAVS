/*
 * 可由上位机修改的透传设置，保存在 Flash（A/B 两页交替），修改后立即生效
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    CFG_MODE_AVS       = 0,     /* a：Fixed + AVS（由 PPS 平移），默认 */
    CFG_MODE_PPS       = 1,     /* b：Fixed + PPS 原样透传 */
    CFG_MODE_AVS_PPS   = 2,     /* c：Fixed + AVS + PPS */
    CFG_MODE_FIXED     = 3,     /* d：仅 Fixed */
} cfg_mode_t;

/* hide_fixed 位 */
#define CFG_HIDE_9V         (1u << 0)
#define CFG_HIDE_12V        (1u << 1)
#define CFG_HIDE_15V        (1u << 2)
#define CFG_HIDE_20V        (1u << 3)

/* flags 位 */
#define CFG_FIX12           (1u << 0)   /* 充电器没有 12V 时，用 PPS（优先）或 AVS 提供 12V Fixed，PDO 位置不够就不提供 */
#define CFG_LOG_PPS         (1u << 1)   /* 记录设备的 PPS 请求（默认不记录：PPS 会频繁调压） */
#define CFG_FE_AVS_2ND      (1u << 2)   /* Lab：充电器查询 Sink_Capabilities_Extended 时声明支持 AVS，触发二次握手 */
#define CFG_BE_AVS_2ND      (1u << 3)   /* Lab：后端先给 PPS，设备声明支持 AVS 后才给 AVS（模拟二次握手） */
#define CFG_FLAGS_MASK      (CFG_FIX12 | CFG_LOG_PPS | CFG_FE_AVS_2ND | CFG_BE_AVS_2ND)

/* 上位机协议直接收发此结构（小端，16 字节） */
typedef struct __attribute__((packed))
{
    uint8_t  mode;          /* cfg_mode_t */
    uint8_t  hide_fixed;    /* CFG_HIDE_*，5V 不可隐藏 */
    uint16_t max_mv;        /* 15000 / 20000 */
    uint16_t max_ma;        /* 500 ~ 3000，50mA 步进 */
    uint8_t  ovp_pct;       /* 1 ~ 20 */
    uint8_t  uvp_pct;       /* 1 ~ 20 */
    uint16_t ocp_ma;        /* 500 ~ 5000 */
    uint16_t ocp_ms;        /* 1 ~ 1000 */
    uint8_t  flags;         /* CFG_* 标志 */
    uint8_t  reserved[3];
} cfg_t;

void cfg_init(void);                    /* 从 Flash 读取，无效则用默认值 */
const cfg_t *cfg(void);
void cfg_defaults(cfg_t *c);
bool cfg_valid(const cfg_t *c);
bool cfg_set(const cfg_t *c);           /* 校验并立即生效，Flash 保存排队 */
bool cfg_save_pending(void);
bool cfg_flush_step(void);              /* store 模块在 PD 空闲时调用；做了一次 Flash 操作返回 true */
