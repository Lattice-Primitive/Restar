#include "drv_led.h"

#ifdef USE_STDPERIPH_DRIVER 

#endif /*USE_STDPERIPH_DRIVER*/

#ifdef USE_HAL_DRIVER
#include "stm32f4xx_ll_gpio.h"

void LED_On(uint_fast16_t led)
{
    LL_GPIO_ResetOutputPin(GPIOG,(led&0x00ff)<<1);//共阳极接法，低电平点亮
    LL_GPIO_ResetOutputPin(GPIOF,(led&0x0100)<<6);
    LL_GPIO_ResetOutputPin(GPIOE,(led&0x0200)<<2);
}

void LED_Off(uint_fast16_t led)
{
    LL_GPIO_SetOutputPin(GPIOG,(led&0xff)<<1);
    LL_GPIO_SetOutputPin(GPIOF,(led&0x0100)<<6);
    LL_GPIO_SetOutputPin(GPIOE,(led&0x0200)<<2);
}

void LED_Toggle(uint_fast16_t led)
{
    LL_GPIO_TogglePin(GPIOG,(led&0xff)<<1);
    LL_GPIO_TogglePin(GPIOF,(led&0x0100)<<6);
    LL_GPIO_TogglePin(GPIOE,(led&0x0200)<<2);
}

#endif /*USE_HAL_DRIVER*/
