#include "pd_phy.h"
#include "timebase.h"

#define T_RECEIVE_US        1100    /* tReceive：等待 GoodCRC 的时间 */
#define N_RETRY_COUNT       2       /* PD3.0 nRetryCount */

/* 参考手册 §15.2.15：端口寄存器位 */
#define PORT_CE             (1u << 7)
#define PORT_LVE            CC_LVE

/* 参考手册 §20.3.1：EXTEN_CTLR0 */
#define EXTEN_WR_LOCK       (1u << 15)

/* 参考手册 §2.4.1：PWR_CTLR.ISINKEN 在 bit10（SDK 头文件 0x0100 有误，以 RM/SVD 为准） */
#define PWR_ISINKEN         (1u << 10)

pd_phy_t pd_phy_be;
pd_phy_t pd_phy_fe;

volatile uint8_t *pd_phy_port_reg(pd_phy_t *p, uint8_t cc_sel)
{
    return cc_sel ? &p->regs->PORT_CC2 : &p->regs->PORT_CC1;
}

static void phy_set_rx(pd_phy_t *p)
{
    USBPD_TypeDef *r = p->regs;

    r->PORT_CC1 &= ~PORT_LVE;
    r->PORT_CC2 &= ~PORT_LVE;

    r->CONFIG |= PD_ALL_CLR;
    r->CONFIG &= ~PD_ALL_CLR;
    r->CONFIG |= IE_TX_END | IE_RX_ACT | IE_RX_RESET | PD_DMA_EN;

    r->DMA = (uint16_t)(uint32_t)p->rx_buf;
    r->BMC_CLK_CNT = UPD_TMR_RX_48M;

    r->CONTROL &= ~PD_TX_EN;
    r->CONTROL |= BMC_START;
}

/* 启动发送并忙等 TX_END（调用方负责屏蔽本端口中断） */
static void phy_tx_blocking(pd_phy_t *p, uint8_t len, uint8_t sop)
{
    USBPD_TypeDef *r = p->regs;

    *pd_phy_port_reg(p, p->cc_sel) |= PORT_LVE;

    r->CONFIG |= PD_ALL_CLR;
    r->CONFIG &= ~PD_ALL_CLR;
    r->CONFIG |= PD_DMA_EN;

    r->TX_SEL = sop;
    r->DMA = (uint16_t)(uint32_t)p->tx_buf;
    r->BMC_CLK_CNT = UPD_TMR_TX_48M;
    r->BMC_TX_SZ = len;

    r->STATUS = IF_TX_END;
    r->CONTROL |= PD_TX_EN;
    r->CONTROL |= BMC_START;

    uint32_t start = micros();
    while (((r->STATUS & IF_TX_END) == 0) && ((uint32_t)(micros() - start) < 2000))
        ;
    r->STATUS = IF_TX_END;

    phy_set_rx(p);
}

void pd_phy_hw_init(void)
{
    RCC_HBPeriphClockCmd(RCC_HBPeriph_USBPD0 | RCC_HBPeriph_USBPD1, ENABLE);
    RCC_PB1PeriphClockCmd(RCC_PB1Periph_PWR, ENABLE);
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOA | RCC_PB2Periph_AFIO, ENABLE);

    /* 使用 USBPD 功能时需开启 ISINKEN（RM §15.2.15 注） */
    PWR->CTLR |= PWR_ISINKEN;

    /* CC 引脚高阈值输入 */
    EXTEN->EXTEN_KEYR = EXTEN_KEY1;
    EXTEN->EXTEN_KEYR = EXTEN_KEY2;
    EXTEN->EXTEN_CTLR0 |= EXTEN_USBPD0_CC_HVT | EXTEN_USBPD1_CC_HVT | EXTEN_USBPD0_CC_REF | EXTEN_USBPD1_CC_REF;
    EXTEN->EXTEN_CTLR0 |= EXTEN_WR_LOCK;

    GPIO_InitTypeDef gpio = {0};
    gpio.GPIO_Pin = BE_CC_PINS | FE_CC_PIN;
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);
}

void pd_phy_init(pd_phy_t *p, USBPD_TypeDef *regs, IRQn_Type irqn, uint8_t power_role, uint8_t data_role)
{
    memset(p, 0, sizeof(*p));
    p->regs = regs;
    p->irqn = irqn;
    p->power_role = power_role;
    p->data_role = data_role;
    p->revision = PD_REV_30;

    regs->CONFIG = 0;
    phy_set_rx(p);

    NVIC_SetPriority(irqn, 0x00);
    NVIC_EnableIRQ(irqn);
}

void pd_phy_set_cc(pd_phy_t *p, uint8_t cc_sel)
{
    p->cc_sel = cc_sel;
    if (cc_sel)
        p->regs->CONFIG |= CC_SEL;
    else
        p->regs->CONFIG &= ~CC_SEL;
}

void pd_phy_reset_protocol(pd_phy_t *p)
{
    NVIC_DisableIRQ(p->irqn);
    p->tx_msg_id = 0;
    p->q_head = p->q_tail = 0;
    p->goodcrc_rcvd = false;
    p->revision = PD_REV_30;
    NVIC_EnableIRQ(p->irqn);
}

bool pd_phy_rx_pop(pd_phy_t *p, pd_rx_msg_t *out)
{
    if (p->q_tail == p->q_head)
        return false;
    *out = p->q[p->q_tail];
    p->q_tail = (p->q_tail + 1) % PD_RX_QUEUE_LEN;
    return true;
}

bool pd_phy_send(pd_phy_t *p, uint8_t msg_type, uint8_t num_objs, const uint32_t *objs)
{
    uint16_t header = pd_build_header(msg_type, num_objs, p->tx_msg_id, p->power_role, p->data_role, p->revision);
    uint8_t len = 2 + num_objs * 4;

    for (uint8_t attempt = 0; attempt <= N_RETRY_COUNT; attempt++)
    {
        NVIC_DisableIRQ(p->irqn);
        p->tx_buf[0] = header & 0xFF;
        p->tx_buf[1] = header >> 8;
        for (uint8_t i = 0; i < num_objs; i++)
        {
            pd_put_u32(&p->tx_buf[2 + i * 4], objs[i]);
        }
        p->goodcrc_rcvd = false;
        phy_tx_blocking(p, len, UPD_SOP0);
        NVIC_EnableIRQ(p->irqn);

        uint32_t start = micros();
        while ((uint32_t)(micros() - start) < T_RECEIVE_US)
        {
            if (p->goodcrc_rcvd && p->goodcrc_id == p->tx_msg_id)
            {
                p->tx_msg_id = (p->tx_msg_id + 1) & 0x7;
                return true;
            }
        }
    }
    /* 未收到 GoodCRC：MessageID 仍然递增，避免对方把下一条当作重发丢弃 */
    p->tx_msg_id = (p->tx_msg_id + 1) & 0x7;
    return false;
}

void pd_phy_send_hard_reset(pd_phy_t *p)
{
    NVIC_DisableIRQ(p->irqn);
    phy_tx_blocking(p, 0, UPD_HARD_RESET);
    NVIC_EnableIRQ(p->irqn);
}

/* 中断：接收完成立即回 GoodCRC，再入队交给主循环 */
static void phy_isr(pd_phy_t *p)
{
    USBPD_TypeDef *r = p->regs;

    if (r->STATUS & IF_RX_ACT)
    {
        uint8_t status = r->STATUS;
        uint16_t rx_len = r->BMC_BYTE_CNT;
        r->STATUS = IF_RX_ACT;

        uint8_t sop = status & BMC_AUX_Mask;
        bool replied = false;
        if (sop == BMC_AUX_SOP0 && rx_len >= 6)
        {
            uint16_t header = p->rx_buf[0] | (p->rx_buf[1] << 8);
            pd_header_t h = pd_parse_header(header);
            uint8_t msg_len = rx_len - 4; /* 去掉 CRC32 */

            if (!h.extended && h.num_objs == 0 && h.msg_type == MSG_TYPE_GoodCRC)
            {
                p->goodcrc_id = h.msg_id;
                p->goodcrc_rcvd = true;
            }
            else
            {
                uint8_t next = (p->q_head + 1) % PD_RX_QUEUE_LEN;
                if (next != p->q_tail && msg_len <= PD_MAX_MSG_LEN)
                {
                    p->q[p->q_head].len = msg_len;
                    memcpy(p->q[p->q_head].data, p->rx_buf, msg_len);
                    p->q_head = next;
                }

                uint16_t goodcrc = pd_build_header(MSG_TYPE_GoodCRC, 0, h.msg_id, p->power_role, p->data_role, h.revision);
                p->tx_buf[0] = goodcrc & 0xFF;
                p->tx_buf[1] = goodcrc >> 8;
                phy_tx_blocking(p, 2, UPD_SOP0);
                replied = true;
            }
        }
        if (!replied)
        {
            phy_set_rx(p);
        }
    }

    if (r->STATUS & IF_TX_END)
    {
        r->STATUS = IF_TX_END;
        phy_set_rx(p);
    }

    if (r->STATUS & IF_RX_RESET)
    {
        uint8_t status = r->STATUS;
        r->STATUS = IF_RX_RESET;
        if ((status & BMC_AUX_Mask) == BMC_AUX_SOP1_HRST)
        {
            p->hard_reset_rcvd = true;
        }
        phy_set_rx(p);
    }
}

void USBPD0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USBPD0_IRQHandler(void)
{
    phy_isr(&pd_phy_be);
}

void USBPD1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USBPD1_IRQHandler(void)
{
    phy_isr(&pd_phy_fe);
}
