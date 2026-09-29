/*
 * UFCS 物理层（充电设备 / Sink 一侧）：前端 D+ = PB0（UART1 RX），D− = PB1（UART1 TX，重映射 011）
 *
 * 与 USB HID 共用 PB0/PB1，互斥：本模块只在 USBFS 未启动时使用。
 *  - DCP 判别：D− 输出 0.6V，D+ 用比较器看是否跟随（DCP 短接 D+/D−）
 *  - 握手：D− 高 2ms / 低 8ms / 高 2ms / 低 8ms，期间 D+ 被充电器拉高即成功，然后切到 UART
 *  - 接收：EXTI 量训练字节 0xAA 的边沿间隔，得到本包实际波特率，再用 UART 收其余字节
 *  - 发送：按标称波特率（115200/57600/38400）8N2，先发训练字节
 */
#pragma once

#include "ufcs_codec.h"

#define UFCS_BAUD_115200        0
#define UFCS_BAUD_57600         1
#define UFCS_BAUD_38400         2

typedef struct
{
    uint8_t  len;
    uint8_t  baud;              /* 训练字节判定的档位 UFCS_BAUD_* */
    uint16_t bit_ticks;         /* 实测位宽，12MHz 计数（115200 ≈ 104） */
    uint32_t t_us;              /* 收完时的 micros() */
    uint8_t  data[UFCS_MAX_PKT];
} ufcs_rx_pkt_t;

typedef struct
{
    uint16_t train_ok;          /* 训练字节合格次数 */
    uint16_t train_bad;         /* 边沿间隔不合格（干扰、半包或波特率超差） */
    uint16_t frame_err;         /* 数据字节成帧/溢出错误 */
    uint16_t timeout;           /* 帧内/帧间超时（tFrameReceive） */
    uint16_t overflow;          /* 接收队列满 */
    uint16_t hard_reset_rx;     /* 收到 D+ 长低（硬复位） */
} ufcs_phy_stats_t;

/* DCP 判别（约 20ms，阻塞）。flags：bit0 = D+ 跟随 D−（短接），bit1 = 撤去驱动后 D+ 回到低（基线正常） */
bool ufcs_phy_dcp_detect(uint8_t *flags);

/* 阻塞等待期间调用的回调（喂看门狗等） */
void ufcs_phy_set_yield(void (*fn)(void));

/* 电平扫描（DCP 判别之外的诊断）：用比较器扫 6 位 DAC（每档 3.3V/64 ≈ 51.6mV），得到引脚电压档位（0~63，引脚 < 档位×51.6mV）
 * lv[0] = D+ 悬空，lv[1] = D− 悬空，lv[2] = D− 驱动 0.6V 时的 D+，lv[3] = D+ 驱动 0.6V 时的 D− */
void ufcs_phy_scan(uint8_t lv[4]);

/* 握手（最多 3 次，阻塞约 30~90ms）。成功后已切到 UART 模式并可收发 */
bool ufcs_phy_handshake(uint8_t *tries);

/* 退出 UFCS，D+/D− 回到高阻，可以启动 USB HID */
void ufcs_phy_release(void);
bool ufcs_phy_active(void);

void ufcs_phy_set_baud(uint8_t baud);
uint8_t ufcs_phy_baud(void);

/* 发送一个整包（不含训练字节，函数内自动加）；正在收/发时返回 false */
bool ufcs_phy_send(const uint8_t *pkt, uint8_t len);
bool ufcs_phy_tx_busy(void);
uint32_t ufcs_phy_tx_end_us(void);      /* 最近一包发送完成的 micros() */

bool ufcs_phy_recv(ufcs_rx_pkt_t *p);   /* 取出一个完整的包 */
bool ufcs_phy_rx_busy(void);            /* 正在接收一包 */

void ufcs_phy_process(void);            /* 主循环调用：帧超时、硬复位检测 */
uint16_t ufcs_phy_take_reset_len(void);   /* 硬复位低电平持续时间 µs（复位结束后才有值，读后清零；0 = 尚无） */
bool ufcs_phy_take_reset(void);         /* 检测到充电器硬复位（读后清除） */
void ufcs_phy_hard_reset(void);         /* 发送充电设备硬复位（D− 低 2.5ms，阻塞），之后仍在 UART 模式 */

const ufcs_phy_stats_t *ufcs_phy_stats(void);
