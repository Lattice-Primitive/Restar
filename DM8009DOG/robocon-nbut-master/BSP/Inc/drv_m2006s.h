/**
 * @file drv_m2006s.h 驱动层 m2006及类似电机
 * @author Zhiyuan Mao(2019th):HAL
 * @note 发送函数封装，
*/
#ifndef __DRV_M2006S_H
#define __DRV_M2006S_H

#ifdef __cplusplus
extern "C"
{
#endif /*__cplusplus*/

#include <stdint.h>

#define M2006S_01 (0x201)
#define M2006S_02 (0x202)
#define M2006S_03 (0x203)
#define M2006S_04 (0x204)
#define M2006S_05 (0x205)
#define M2006S_06 (0x206)
#define M2006S_07 (0x207)
#define M2006S_08 (0x208)

#define M2006S_01_04 (0x200)
#define M2006S_05_08 (0x1ff)

typedef struct
{
    uint16_t Rx_StdId;
    uint16_t Rotate;
    uint16_t Speed;
    uint16_t Current;
    uint16_t Temp;
} M2006S_Receive_Data_t;

typedef struct
{
    uint16_t Tx_StdId;
    uint16_t Current1;
    uint16_t Current2;
    uint16_t Current3;
    uint16_t Current4;
} M2006S_Transmit_Data_t;

void M2006S_SetCurrent(uint16_t tx_id,M2006S_Transmit_Data_t *tx_data);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif 
