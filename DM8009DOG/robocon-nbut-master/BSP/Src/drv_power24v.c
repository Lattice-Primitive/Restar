#include "drv_power24v.h"

#ifdef USE_STDPERIPH_DRIVER /*标准库的初始化*/
#include "stm32f4xx.h"

void Power24v_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOH, &GPIO_InitStructure);
}

void Power24v_On(Power24V_Switch_t sw)
{
    GPIOH->BSRRL = sw << 2; //置位
}

void Power24v_Off(Power24V_Switch_t sw)
{
    GPIOH->BSRRH = sw << 2; //复位
}

#endif /*USE_STDPERIPH_DRIVER*/

#ifdef USE_HAL_DRIVER
#include "stm32f4xx_ll_gpio.h"

void Power24v_On(Power24V_Switch_t sw)
{
    LL_GPIO_SetOutputPin(GPIOH,1 << (2+sw));
}

void Power24v_Off(Power24V_Switch_t sw)
{
    LL_GPIO_ResetOutputPin(GPIOH,1 << (2+sw));
}

#endif /*USE_HAL_DRIVER*/
