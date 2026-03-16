/**
 * @file drv_wt61c.h 驱动层 WT61C陀螺仪
 * @author Zhiyuan Mao(2019th):HAL
 * @note 用于驱动WT61C并获取数据
*/
#ifndef __DRV_WT61C_H
#define __DRV_WT61C_H

#ifdef __cplusplus
extern "C"
{
#endif /*__cplusplus*/

#include <stdint.h>

/*WT61C指令集*/
typedef enum
{
    ZANGLE_INIT=0x52,/*使Z轴角度归零*/
    ACCEL_CORRECT=0x67,/*校准加速度零偏*/
    SLEEP_WAKE=0x60,/*休眠及解休眠*/
    BAUD_115200=0x63,/*波特率115200，回传速率100HZ*/
    BAUD_9600=0x64,/*波特率9600，回传速率20HZ*/
    HORIZON_SET=0x65,/*水平安装*/
    VERTICAL_SET=0x66/*垂直安装*/
}WT61C_Cmd_t;

/*WT61C数据类型集*/
typedef enum
{
    WT61C_ACCEL,/*加速度数据*/
    WT61C_GYRO,/*角速度数据*/
    WT61C_ANGLE,/*角度数据*/
    WT61C_TEMP/*温度数据*/
}WT61C_DataType_t;

/*波特率选择*/
typedef enum
{
    WT61C_BAUD_9600,/*波特率9600*/
    WT61C_BAUD_115200/*波特率115200*/
}WT61C_BaudRate_t;

/*3轴数据结构体(float)*/
typedef struct
{
    uint16_t x;/*x轴数据*/
    uint16_t y;/*y轴数据*/
    uint16_t z;/*z轴数据*/
}WT61C_Axis_ShortData_t;

#if __FPU_USED /*浮点运算器是否打开*/

/*3轴数据结构体(float)*/
typedef struct
{
    float x;/*x轴数据*/
    float y;/*y轴数据*/
    float z;/*z轴数据*/
}WT61C_Axis_FloatData_t;

#endif /*__FPU_USED*/

/**
 * @brief 初始化，选择用于陀螺仪的UART模块，并执行初始化命令
 * @param wt61c_uart (void *) UART模块地址(比如USART1之类)
 * @param baud_rate (WT61C_BaudRate_t)波特率选择
 * @return NULL (void)
*/
extern void WT61C_Init(void *wt61c_uart, WT61C_BaudRate_t baud_rate);

/**
 * @brief WT61C发送指令，一般情况不需要使用
 * @param cmd (WT61C_Cmd_t) WT61C指令
 * @return NULL (void)
*/
extern void WT61C_SendCmd(WT61C_Cmd_t cmd);

/**
 * @brief WT61C接收数据回调函数，用于每次接受到一个数据后，该函数会自动判断内容并在内部处理
 * @note 可以用于中断与DMA
 * @param get_data (uint8_t) 获得到的数据
 * @return NULL (void)
*/
extern void WT61C_GetData_Callback(uint8_t get_data);

/**
 * @brief 获取处理后的数据(16位整型)
 * @param type (WT61C_DataType_t)需要的WT61C数据类型
 * @param sdata (WT61C_Axis_ShortData_t*) 储存数据的3轴数据结构体
 * @return NULL (void)
*/
extern void WT61C_GetShortData(WT61C_DataType_t type, WT61C_Axis_ShortData_t* sdata);

#if __FPU_USED /*浮点运算器是否打开*/

/**
 * @brief 获取处理后的数据(32位浮点)
 * @param type (WT61C_DataType_t)需要的WT61C数据类型
 * @param sdata (WT61C_Axis_FloatData_t*) 储存数据的3轴数据结构体
 * @return NULL (void)
*/
void WT61C_GetFloatData(WT61C_DataType_t type, WT61C_Axis_FloatData_t* fdata);

#endif /*__FPU_USED*/

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /*__DRV_WT61C_H*/
