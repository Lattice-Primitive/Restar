/**
 * @file drv_delay.h 驱动层 延时
 * @author Zhiyuan Mao(2019th):HAL Aoming Yu(2016th):STD
 * @note 延时函数,保留了一些老的写法,HAL库ms级延时建议HAL_Delay()
*/
#ifndef __DRV_DELAY_H
#define __DRV_DELAY_H

#ifdef __cplusplus
extern "C"
{
#endif /*__cplusplus*/

#include <stdint.h>

/*老写法保留*/
#define delay_ms(nms) Delay_Ms(nms)
#define delay_us(ums) Delay_Us(ums)

    typedef enum
    {
        Ticking = 0,
        Stop = 1,
    } Delay_Countdown_Status_t;

#ifdef USE_STDPERIPH_DRIVER /*标准库还需要初始化*/
/*systick重装填频率默认值1000hz*/
#define DELAY_TICK_RATE_DEFAULT (1000)

    /**
 * @brief 设置systick功能
 * @param tick_rate_hz (uint32_t)systick溢出频率
 * @return NULL (void)
*/
    extern void Delay_Init(uint32_t tick_rate_hz);
#endif /*USE_STDPERIPH_DRIVER*/

    /**
 * @brief 微秒级延时
 * @param nus (uint32_t) 延时微秒数
 * @return NULL (void)
*/
    extern void Delay_Us(uint32_t nus);

    /**
 * @brief 毫秒级延时
 * @param nms (uint32_t) 延时微秒数
 * @return NULL (void)
*/
extern inline void Delay_Ms(uint32_t nms)
{
    Delay_Us(1000U * nms);
}

/**
 * @brief 启动倒计时
 * @param nTime (uint32_t)倒计时时间设置
 * @return NULL 
*/
void Delay_CountdownBegin(uint32_t nTime);

/**
 * @brief 取消倒计时
 * @param NULL (void)
 * @return NULL (void)
*/
void Delay_CountdownCancel(void);

/**
 * @brief 判断是否倒计时结束
 * @param NULL (void)
 * @return (uint8_t) 是否倒计时结束,0->倒计时未结束或没有倒计时,1->倒计时结束
*/
uint8_t Delay_CountdownIsTimeout(void);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /*__DRV_DELAY_H*/
