/**
 * @file drv_led.h 驱动层 LED
 * @author Zhiyuan Mao(2019th):HAL
 * @note 用于驱动robomaster开发板a型板载LED，开发板上的LED有：
 *         引脚     位号
 * 流水LED PG1~PG8  D1~D8
 * 绿LED   PF14     D9
 * 红LED   PE11     D10
*/
#ifndef __DRV_LED_H
#define __DRV_LED_H

#if STM32F427X | STM32F427xx //基于STM32F427的robomaster开发板a型

#ifdef __cplusplus
extern "C"
{
#endif /*__cplusplus*/

#include <stdint.h>

#define LED_FLOW_1 (uint_fast16_t)(1 << 0) /*PG1,D1*/
#define LED_FLOW_2 (uint_fast16_t)(1 << 1) /*PG2,D2*/
#define LED_FLOW_3 (uint_fast16_t)(1 << 2) /*PG3,D3*/
#define LED_FLOW_4 (uint_fast16_t)(1 << 3) /*PG4,D4*/
#define LED_FLOW_5 (uint_fast16_t)(1 << 4) /*PG5,D5*/
#define LED_FLOW_6 (uint_fast16_t)(1 << 5) /*PG6,D6*/
#define LED_FLOW_7 (uint_fast16_t)(1 << 6) /*PG7,D7*/
#define LED_FLOW_8 (uint_fast16_t)(1 << 7) /*PG8,D8*/
#define LED_GREEN  (uint_fast16_t)(1 << 8) /*PF14,D9*/
#define LED_RED    (uint_fast16_t)(1 << 9) /*PE11,D10*/

#ifdef USE_STDPERIPH_DRIVER /*LED初始化用于标准库*/
    /**
 * @brief LED初始化
 * @param NULL (void)
 * @return NULL (void)
*/
extern void LED_Init(void);
#endif /*USE_STDPERIPH_DRIVER*/

/**
 * @brief LED点亮
 * @param led 点亮的LED灯，可选值为 LED_RED LED_GREEN LED_FLOW_x (x=1~8)
 * @return NULL (void)
*/
extern void LED_On(uint_fast16_t led);

/**
 * @brief LED关闭
 * @param led 点亮的LED灯，可选值为 LED_RED LED_GREEN LED_FLOW_x (x=1~8)
 * @return NULL (void)
*/
extern void LED_Off(uint_fast16_t led);

/**
 * @brief LED反转电平
 * @param led 关闭的LED灯，可选值为 LED_RED LED_GREEN LED_FLOW_x (x=1~8)
 * @return NULL (void)
*/
extern void LED_Toggle(uint_fast16_t led);

/**
 * @brief 选择要亮起的LED，剩下的关闭
 * @param led  点亮的LED灯，可选值为 LED_RED LED_GREEN LED_FLOW_x (x=1~8)
 * @return NULL (void
*/
extern inline void LED_Display(uint_fast16_t led)
{
    LED_On(led);
    LED_Off(~(led&0x03ff));
}

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /*STM32F427X|STM32F427xx*/

#endif /*__DRV_LED_H*/
