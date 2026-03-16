/**
 * @file drv_buzzer 蜂鸣器驱动
 * @author 
 * @note 还没写，等一个有缘人
*/
#ifndef __DRV_BUZZER_H
#define __DRV_BUZZER_H

#ifdef __cplusplus
extern "C"
{
#endif /*__cplusplus*/

#include <stdint.h>
#include "buzzer_freq.h"

typedef struct
{
    void (*Buzzer_Set_Freq)(uint32_t Freq);
    void (*Buzzer_Silent)(void);
    void (*Buzzer_Delay)(uint32_t ticks);
}Buzzer_Operation_t;

/**
 * @brief 蜂鸣器发声
 * @param opt (Buzzer_Operation_t*) 蜂鸣器操作函数
 * @param Freq (uint32_t) 频率，单位为 Hz*10 如 29Hz->290,兼容buzzer_freq.h
 * @param ticks (uint32_t) 持续时间 单位ms，兼容buzzer_freq.h
 * @return (void) NULL
*/
extern void Buzzer_Note(Buzzer_Operation_t* opt,uint32_t Freq,uint32_t ticks);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif // __DRV_BUZZER_H