/**
 * @file operates.h 外设操作结构体模板
 * @author Zhiyuan Mao(2019th) 全平台
 * @note 一个设想，未来希望使用同一套驱动代码用于不同的芯片
*/
#ifndef __OPERATES_H
#define __OPERATES_H

#ifdef __cplusplus
extern "C"
{
#endif /*__cplusplus*/

#include <stdint.h>

typedef struct __GPIO_Operate
{
    void *module;
    void (*Init)(void);
    void (*Low)(struct __GPIO_Operate* oprt,uint32_t pin);
    void (*High)(struct __GPIO_Operate* oprt,uint32_t pin);
    void (*Toggle)(struct __GPIO_Operate* oprt,uint32_t pin);
    void (*Lock)(struct __GPIO_Operate* oprt,uint32_t pin);
    uint32_t (*Read)(struct __GPIO_Operate* oprt);
}GPIO_Operate;

typedef struct __I2C_Operate
{
    void *module;
    void (*Init)(struct __I2C_Operate* oprt);
    void (*Write)(struct __I2C_Operate* oprt,uint8_t* data,uint32_t data_num);
    void (*Read)(struct __I2C_Operate* oprt,uint8_t* data,uint32_t data_num);
    void (*Mem_Write)(struct __I2C_Operate* oprt,uint8_t slave_addr,uint8_t reg_addr,uint8_t* pdata,uint32_t data_num);
    void (*Mem_Read)(struct __I2C_Operate* oprt,uint8_t slave_addr,uint8_t reg_addr,uint8_t* pdata,uint32_t data_num);
}I2C_Operate;

typedef struct __SPI_Operate
{
    void *module;
    void (*Init)(struct __SPI_Operate* oprt);
    void (*Write)(struct __SPI_Operate* oprt,uint8_t* data,uint32_t data_num);
    void (*Read)(struct __SPI_Operate* oprt,uint8_t* data,uint32_t data_num);
}SPI_Operate;

typedef struct __UART_Operate
{
    void *module;
    void (*Init)(struct __UART_Operate* oprt);
    void (*Write)(struct __UART_Operate* oprt,uint8_t* data,uint32_t data_num);
    void (*Read)(struct __UART_Operate* oprt,uint8_t* data,uint32_t data_num);
}UART_Operate;

typedef struct __PWM_Operate
{
    void *module;
    void (*Init)(struct __PWM_Operate* oprt);
    void (*Start)(struct __PWM_Operate* oprt);
    void (*Write)(struct __PWM_Operate* oprt,uint32_t new_value);
    void (*Stop)(struct __PWM_Operate* oprt,uint32_t new_value);
}PWM_Operate;

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /*__OPERATES_H*/
