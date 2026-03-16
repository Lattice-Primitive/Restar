#include "drv_delay.h"

#ifdef USE_STDPERIPH_DRIVER /*标准库的初始化*/
#include "stm32f4xx.h"

static uint32_t fac_us = 0;

void Delay_Init(uint32_t tick_rate_hz)
{
    uint32_t reload = 0;
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
    fac_us = SystemCoreClock / 8000000;

    if (tick_rate_hz == 0)
    {
        tick_rate_hz = 1000;
    }

    reload = SystemCoreClock / tick_rate_hz / 8;
    reload--;

    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
    SysTick->LOAD = reload;
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}
#endif /*USE_STDPERIPH_DRIVER*/

#ifdef USE_HAL_DRIVER /*HAL库对Systick的引用*/
#include "stm32f4xx_hal.h"
#endif /*USE_HAL_DRIVER*/

static uint32_t fac_us;
static volatile uint32_t sysTickCnt;
static Delay_Countdown_Status_t Countdown_status=Stop;

void Delay_Us(uint32_t nus)
{
    uint32_t ticks = 0;
    uint32_t told = 0;
    uint32_t tnow = 0;
    uint32_t tcnt = 0;
    uint32_t reload = 0;
    fac_us=SystemCoreClock / (1000000U) * uwTickFreq;
    reload = SysTick->LOAD;//reload:重装填值
    ticks = nus * fac_us;//ticks:总共要经过的tick数
    told = SysTick->VAL;//told:上一次记录值
    while (1)
    {
        tnow = SysTick->VAL;//tnow:目前值,systick为向下计数
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;//tcnt:总计数值
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break;
            }
        }
    }
}

void Delay_CountdownBegin(uint32_t nTime)
{
    // 这里设置为1ms中断一次
	sysTickCnt = nTime + HAL_GetTick();
    Countdown_status = Ticking;
}

void Delay_CountdownCancel(void)
{
	// systick 定时器失能
	Countdown_status = Stop;
}

uint8_t Delay_CountdownIsTimeout(void)
{
    if(Countdown_status == Ticking)
        return sysTickCnt >= HAL_GetTick();
    return 0;
}
