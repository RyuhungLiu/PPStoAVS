/*
 * 前端 Sink 策略引擎（USBPD1，PA3/CC4R）
 */
#pragma once

#include "pd_defs.h"

typedef struct
{
    uint8_t    pos;         /* 1-based，对应充电器 Source_Capabilities 中的位置 */
    pdo_type_t type;        /* FPDO 或 PPS_PDO */
    uint16_t   mv;
    uint16_t   ma;
} fe_target_t;

typedef enum
{
    FE_REQ_IDLE,
    FE_REQ_BUSY,
    FE_REQ_OK,
    FE_REQ_FAIL,
} fe_req_status_t;

void fe_init(void);
void fe_process(void);

bool fe_is_ready(void);         /* 已有显式合约且无进行中的请求 */
bool fe_is_legacy(void);        /* 充电器不支持 PD */
uint16_t fe_legacy_current_ma(void);

const pdo_t *fe_caps(uint8_t *num);
bool fe_caps_unconstrained(void);
const fe_target_t *fe_contract(void);

#ifdef FE_DIAG
extern uint16_t fe_dbg_v[6];
#endif

bool fe_request(const fe_target_t *t);  /* 由协议桥发起；忙时返回 false */
fe_req_status_t fe_request_status(void);
