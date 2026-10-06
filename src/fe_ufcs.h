/*
 * 前端 UFCS 适配：把 UFCS 充电器的输出模式翻译成合成的 PD 能力（Fixed 5/9/15/20V + 每个模式一个 PPS），
 * 把协议桥的前端请求翻译成 UFCS Request。协议桥和后端不知道前端是哪种协议。
 * fe_sink.c 的公开接口在 UFCS 激活时转到这里。
 */
#pragma once

#include "fe_sink.h"

bool feu_start(void);                   /* 阻塞：握手 + Ping；成功后激活（此时不应启动 USB HID 和 PD 前端） */
bool feu_start_qc(void);                /* 阻塞约 1.6~2.1s：QC2.0/3.0 握手与探测；成功后激活（此时不应启动 USB HID 和 PD 前端） */
bool feu_active(void);
bool feu_process(void);                 /* false = 会话丢失且重连失败，调用方改用 PD 前端 */
void feu_stop(void);

bool feu_caps_available(void);
uint32_t feu_contract_rdo(void);
bool feu_flash_safe(void);
uint8_t feu_state_code(void);
bool feu_is_ready(void);
const pdo_t *feu_caps(uint8_t *num);
const fe_target_t *feu_contract(void);
bool feu_request(const fe_target_t *t);
fe_req_status_t feu_request_status(void);
