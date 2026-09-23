#include "power_sw.h"
#include "board.h"
#include "timebase.h"

/* 参考手册 §6.3 AFIO_PCFR1[11:10] TIM2_RM */
#define AFIO_TIM2_RM_Mask       (3u << 10)
#define AFIO_TIM2_RM_10         (2u << 10)

/* 参考手册 §10.4 TIM2 */
#define TIM2_CH2_PWMOUT_EN      (1u << 12)
#define TIM_OCxM_PWM1           0x6
#define DTCR_DT2_P              (1u << 4)
#define DTCR_DT1N_P             (1u << 3)
#define DTCR_OC2N_EN            (1u << 1)
#define DTCR_OC1N_EN            (1u << 0)

static bool sw_on = false;

void power_sw_early_off(void)
{
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOB, ENABLE);
    GPIO_SetBits(GATE_GPIO_PORT, GATE_GPIO_PIN);

    GPIO_InitTypeDef gpio = {0};
    gpio.GPIO_Pin = GATE_GPIO_PIN;
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GATE_GPIO_PORT, &gpio);
    GPIO_SetBits(GATE_GPIO_PORT, GATE_GPIO_PIN);
    sw_on = false;
}

/*
 * DS2 §4（三）：
 *  1) CH2_PWMOUT_EN=1，TIM2_CH2 从 PC5 输出
 *  2) PWM 周期 1~5us
 *  3) CH1CVR = CH2CVR，DT2_P=1
 *  4) DT2 约 100ns，OC2N_EN=1（PC5/PB14 带死区互补）；OC1N_EN=1、DT1N_P=1（PB12/PB14 不带死区互补）
 *  5) PC5、PB14、PB12 复用推挽
 */
void power_sw_hvcp_start(void)
{
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOB | RCC_PB2Periph_GPIOC | RCC_PB2Periph_AFIO, ENABLE);
    RCC_PB1PeriphClockCmd(RCC_PB1Periph_TIM2, ENABLE);

    AFIO->PCFR1 = (AFIO->PCFR1 & ~AFIO_TIM2_RM_Mask) | AFIO_TIM2_RM_10;

    TIM2->CTLR1 = 0;
    TIM2->PSC = 0;
    TIM2->ATRLR = HVCP_PWM_PERIOD_TICKS - 1;
    TIM2->CH1CVR = HVCP_PWM_PERIOD_TICKS / 2;
    TIM2->CH2CVR = HVCP_PWM_PERIOD_TICKS / 2;
    TIM2->CHCTLR1 = (TIM_OCxM_PWM1 << 4) | (1u << 3) |      /* OC1M=PWM1，OC1PE */
                    (TIM_OCxM_PWM1 << 12) | (1u << 11);     /* OC2M=PWM1，OC2PE */
    TIM2->TIM2_DTCR = (HVCP_DEADTIME_DT2 << 12) | DTCR_DT2_P | DTCR_DT1N_P | DTCR_OC2N_EN | DTCR_OC1N_EN;
    TIM2->CCER = (1u << 0) | (1u << 4) | (1u << 8) | (1u << 12); /* CC1E~CC4E（互补输出复用通道 3/4） */
    TIM2->SWEVGR = 1;                                              /* UG：装载预装值 */

    GPIO_InitTypeDef gpio = {0};
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_14;
    GPIO_Init(GPIOB, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_5;
    GPIO_Init(GPIOC, &gpio);

    TIM2->CTLR1 = TIM2_CH2_PWMOUT_EN | (1u << 7) /* ARPE */ | (1u << 0) /* CEN */;

    delay_ms(HVCP_STARTUP_MS);
}

void power_sw_set(bool on)
{
    if (on)
        GPIO_ResetBits(GATE_GPIO_PORT, GATE_GPIO_PIN);  /* 高阻：R3 把栅极拉到 VHVCP */
    else
        GPIO_SetBits(GATE_GPIO_PORT, GATE_GPIO_PIN);    /* 拉低：关断 */
    sw_on = on;
}

bool power_sw_is_on(void)
{
    return sw_on;
}
