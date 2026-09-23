#include "timebase.h"
#include "board.h"

/* SysTick：HCLK 计数、向上计数、自动重装，1ms 中断一次 */
#define STK_STE     (1u << 0)
#define STK_STIE    (1u << 1)
#define STK_STCLK   (1u << 2)
#define STK_STRE    (1u << 3)

static volatile uint32_t tick_ms = 0;
static uint32_t ticks_per_us = 48;

void SysTick_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void SysTick_Handler(void)
{
    SysTick->SR = 0;
    tick_ms++;
}

void timebase_init(void)
{
    ticks_per_us = SystemCoreClock / 1000000;
    SysTick->CTLR = 0;
    SysTick->SR = 0;
    SysTick->CNT = 0;
    SysTick->CMP = SystemCoreClock / 1000 - 1;
    SysTick->CTLR = STK_STE | STK_STIE | STK_STCLK | STK_STRE;
    NVIC_SetPriority(SysTick_IRQn, 0xF0);
    NVIC_EnableIRQ(SysTick_IRQn);
}

uint32_t millis(void)
{
    return tick_ms;
}

uint32_t micros(void)
{
    uint32_t ms, cnt, pending;
    do
    {
        ms = tick_ms;
        cnt = SysTick->CNT;
        pending = SysTick->SR & 1;
    } while (ms != tick_ms);
    /* 在更高优先级中断里调用时 SysTick 中断无法执行：计数已回卷但 tick_ms 未加，手动补 1ms */
    if (pending && cnt < SysTick->CMP / 2)
    {
        ms++;
    }
    return ms * 1000 + cnt / ticks_per_us;
}

void delay_us(uint32_t us)
{
    uint32_t start = micros();
    while ((uint32_t)(micros() - start) < us)
        ;
}

void delay_ms(uint32_t ms)
{
    uint32_t start = millis();
    while ((uint32_t)(millis() - start) < ms)
        ;
}
