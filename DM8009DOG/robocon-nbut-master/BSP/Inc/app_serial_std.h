/**
 * @file app_serial.h 串口标准IO重定向
 * @author Zhiyuan Mao(2019th) (全平台)
 * @brief 用于全平台的标准IO重定向，适用于串口，也可用于其他IO设备比如OLED
 * 调用scanf()在串口中输入数据时，必须以空格结束，否则无法完成发送
*/
#ifndef __APP_SERIAL_STD_H
#define __APP_SERIAL_STD_H

#ifdef __cplusplus
extern "C"
{
#endif /*__cplusplus*/

#include <stdint.h>
#include <stdio.h>

typedef struct
{
    /* @brief 输入字符
     * @param (void) NULL
     * @return (uint8_t) 接收到的字符*/
    uint8_t (*Serial_Std_Getc)(void);
    /* @brief 输出字符 
     * @param ch (uint8_t) 要输出的字符
     * @return (uint8_t) 输出的字符，直接把输入的字符作为返回*/
    uint8_t (*Serial_Std_Putc)(uint8_t ch);
}Serial_Std_operation_t;

/**
 * @brief 串口标准IO重定向初始化
 * @param opt (Serial_Std_operation_t*) 串口标准IO重定向操作结构体
 * @return (void) NULL
*/
extern void Serial_Std_Init(Serial_Std_operation_t* opt);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif // __APP_SERIAL_STD_H
