/*
 * HiSilicon D− 单线物理层（AFC / FCP / SCP 共用，见 hisi_phy.c）。调用前须已完成 qc_handshake（D+ 保持 0.6V）。
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define HP_UI           160u
#define HP_MAX_EDGES    64u

extern uint16_t hp_edges[HP_MAX_EDGES];     /* 最近一次 hp_capture 的电平持续 µs（逐段交替） */
extern uint8_t hp_n_edges, hp_first_level;

void hp_out(bool high);                     /* 驱动 D− 高（≈3.25V）/ 低 */
void hp_mping(void);                        /* 主机 Ping：D− 高 16 UI */
int32_t hp_sping(void);                     /* 从机 Ping：返回高电平 µs，< 0 = 没有 / 太短 / 太长 */
void hp_send_byte(uint8_t d);               /* 发送一个字节（关中断约 1.8ms） */
void hp_capture(uint32_t ms);               /* 记录 D− 跳变 */
uint8_t hp_decode(uint8_t *out, uint8_t max, uint8_t *bad);    /* 从记录解出从机字节；bad 第 k 位 = 第 k 字节校验错 */
/* 一次事务：Mping → Sping → 发 tx[n] → Mping → 记录从机回应 cap_ms → Mping → Sping；返回记录到的跳变数（只有从机 Ping 为 2） */
uint8_t hp_xfer(const uint8_t *tx, uint8_t n, uint32_t cap_ms, int32_t *s0, int32_t *s1);
