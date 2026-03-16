/**
 * @file drv_power24v.h 驱动层 电源控制
 * @author Zhiyuan Mao(2019th):HAL Aoming Yu(2016th):STD
 * @note 仅用于控制robomaster开发板a型上的4个24v口
 * 4个24V口的引脚编号和位号
 * 引脚 位号
 * PH2  J20
 * PH3  J22
 * PH4  J19
 * PH5  J21
 * \a 4个电源要分别开启，延时一定时间，大约1ms(学长的代码延时709us)
*/

#ifndef __DRV_POWER_24V_H
#define __DRV_POWER_24V_H

#if STM32F427X | STM32F427xx //基于STM32F427的robomaster开发板a型

#ifdef __cplusplus
extern "C"
{
#endif /*__cplusplus*/

#include <stdint.h>

typedef enum
{
    Power24V_Switch1=0, /*PH2  J20*/
    Power24V_Switch2,   /*PH3  J22*/
    Power24V_Switch3,   /*PH4  J19*/
    Power24V_Switch4    /*PH5  J21*/
}Power24V_Switch_t;

#ifdef USE_STDPERIPH_DRIVER /*标准库的初始化*/

    /**
 * @brief 初始化对应的引脚
 * @param NULL (void)
 * @return NULL (void)
*/
    extern void Power24v_Init(void);

#endif /*USE_STDPERIPH_DRIVER*/

    /**
 * @brief 打开指定的24v电源口
 * @param sw (uint8_t) 要打开的24v电源口
 * @return NULL (void)
*/
    extern void Power24v_On(Power24V_Switch_t sw);

    /**
 * @brief 关闭指定的24v电源口
 * @param sw (uint8_t) 要关闭的24v电源口
 * @return NULL (void)
*/
    extern void Power24v_Off(Power24V_Switch_t sw);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /*STM32F427X|STM32F427xx*/

#endif /*__DRV_POWER_24V_H*/
