/*
 * 前端 Sink 策略引擎（USBPD1，PA3/CC4R）
 */
#pragma once

#include "pd_defs.h"

typedef struct
{
    uint8_t    pos;         /* 1-based，对应充电器 Source_Capabilities 中的位置 */
    pdo_type_t type;        /* FPDO、PPS_PDO 或 SPR_AVS_PDO */
    uint16_t   mv;
    uint16_t   ma;
} fe_target_t;

typedef enum
{
    FE_REQ_IDLE,
    FE_REQ_BUSY,
    FE_REQ_OK,
    FE_REQ_FAIL,
    FE_REQ_ABORTED,     /* 被充电器 Soft Reset 或新的能力报文打断，电压未变，可重发 */
} fe_req_status_t;

void fe_init(void);
void fe_process(void);

bool fe_is_ready(void);         /* 已有显式合约且无进行中的请求 */
bool fe_is_legacy(void);        /* 充电器不支持 PD */
uint16_t fe_legacy_current_ma(void);

const pdo_t *fe_caps(uint8_t *num);
bool fe_caps_available(void);
uint32_t fe_contract_rdo(void);     /* 当前前端合约的 RDO，无合约为 0 */
bool fe_flash_safe(void);           /* 空闲且近期不会发保活，可以做 Flash 操作 */
uint8_t fe_state_code(void);
bool fe_caps_unconstrained(void);
const fe_target_t *fe_contract(void);

#ifdef FE_DIAG
extern uint16_t fe_dbg_v[6];
#endif

bool fe_request(const fe_target_t *t);  /* 由协议桥发起；忙时返回 false */
fe_req_status_t fe_request_status(void);
uint8_t fe_request_waits(void);         /* 最近一次请求收到 Wait 的次数 */
uint8_t fe_request_txwait(void);
void fe_ra_apply(void);
bool fe_epr_mode(void);                 /* 前端处于 EPR 模式（能力表含 8~11 号 EPR PDO） */                 /* 按 CFG_FE_EMARKER 呈现/撤除 Ra（上电尽早调用） */        /* 最近一次协议桥请求等 SinkTxOK 的 ms；FE_TXWAIT_TIMEOUT = 超时仍发送 */
#define FE_TXWAIT_TIMEOUT       255
