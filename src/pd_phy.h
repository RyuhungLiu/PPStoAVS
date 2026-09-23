/*
 * USB PD PHY：BMC 收发、GoodCRC 自动应答与重发
 * USBPD0 / USBPD1 双实例
 */
#pragma once

#include "board.h"
#include "pd_defs.h"

#define PD_RX_QUEUE_LEN     4
#define PD_BUF_LEN          36      /* header + 7 个数据对象 + CRC32，4 字节对齐 */

typedef struct
{
    uint8_t len;                    /* 字节数：header + 数据对象，不含 CRC */
    uint8_t data[PD_MAX_MSG_LEN];
} pd_rx_msg_t;

typedef struct
{
    USBPD_TypeDef *regs;
    IRQn_Type irqn;
    uint8_t power_role;             /* 1 = Source */
    uint8_t data_role;              /* 1 = DFP */
    uint8_t revision;               /* 本端发送用的 Spec Revision */
    uint8_t cc_sel;                 /* 0 = PORT_CC1，1 = PORT_CC2 */

    uint8_t tx_msg_id;              /* 下一条待发报文的 MessageID */

    uint8_t rx_buf[PD_BUF_LEN] __attribute__((aligned(4)));
    uint8_t tx_buf[PD_BUF_LEN] __attribute__((aligned(4)));

    volatile uint8_t q_head;
    volatile uint8_t q_tail;
    pd_rx_msg_t q[PD_RX_QUEUE_LEN];

    volatile bool hard_reset_rcvd;
    volatile bool goodcrc_rcvd;
    volatile uint8_t goodcrc_id;
} pd_phy_t;

extern pd_phy_t pd_phy_be;  /* USBPD0：后端 Source */
extern pd_phy_t pd_phy_fe;  /* USBPD1：前端 Sink */

void pd_phy_hw_init(void);
void pd_phy_init(pd_phy_t *p, USBPD_TypeDef *regs, IRQn_Type irqn, uint8_t power_role, uint8_t data_role);
void pd_phy_set_cc(pd_phy_t *p, uint8_t cc_sel);
void pd_phy_reset_protocol(pd_phy_t *p);

bool pd_phy_rx_pop(pd_phy_t *p, pd_rx_msg_t *out);

/* 发送一条 SOP 报文，等待 GoodCRC，失败按 nRetryCount=2 重发；返回是否收到 GoodCRC */
bool pd_phy_send(pd_phy_t *p, uint8_t msg_type, uint8_t num_objs, const uint32_t *objs);
void pd_phy_send_hard_reset(pd_phy_t *p);

volatile uint8_t *pd_phy_port_reg(pd_phy_t *p, uint8_t cc_sel);
