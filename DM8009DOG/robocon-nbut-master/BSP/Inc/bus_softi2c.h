/**
 * @file bus_softi2c.h 总线层 软件i2c
 * @author Zhiyuan Mao(2019th) (全平台适用)
 * @note 由于各大厂商的硬件i2c都不是特别好，于是我就自己写一个，适用于全平台
*/
#ifndef __BUS_SOFTI2C_H
#define __BUS_SOFTI2C_H

#ifdef __cplusplus
extern "C"
{
#endif /*__cplusplus*/

#include <stdint.h>
#include <stddef.h>

/*软i2c使能*/
typedef enum
{
    SoftI2C_Enable=0,
    SoftI2C_Disable=1
}SoftI2C_IsEnable_t;

/*软i2c状态*/
typedef enum
{
    SoftI2C_OK=0,
    SoftI2C_NOK=1
}SoftI2C_IsOK_t;

/*软i2c操作结构体*/
typedef struct
{
    void (*SCL_H)(void);/*SCL拉高电平*/
    void (*SCL_L)(void);/*SCL拉低电平*/
    void (*SDA_H)(void);/*SDA拉高电平*/
    void (*SDA_L)(void);/*SDA拉低电平*/
    void (*SDA_IN)(void);/*SDA改变为写模式*/
    void (*SDA_OUT)(void);/*SDA改变为读模式*/
    uint32_t (*SDA_R)(void);/*SDA读取电平 @return 读出的电平*/
    void (*Delay_Us)(uint32_t tick);/*延时几微秒 @param tick (uint32_t)延时微秒数*/
}SoftI2C_Operate_t;

/**
 * @brief 软件I2C数据传输结构体
 * @param slave_addr (uint8_t) 目标从机地址
 * @param slave_reg (uint8_t) 目标寄存器地址
 * @param data_size (uint32_t) 数据大小
 * @param data (uint8_t*) 数据指针
 * */
typedef struct 
{
    uint8_t slave_addr;
    uint8_t slave_reg;
    uint32_t data_size;
    uint8_t *pdata;
}SoftI2C_RW_Mem_t;

inline SoftI2C_IsEnable_t SoftI2C_IsEnable(SoftI2C_Operate_t* i2c_opt)
{
    if(i2c_opt==NULL) return SoftI2C_Disable;
    if(i2c_opt->SCL_H==NULL) return SoftI2C_Disable;
    if(i2c_opt->SCL_L==NULL) return SoftI2C_Disable;
    if(i2c_opt->SDA_H==NULL) return SoftI2C_Disable;
    if(i2c_opt->SDA_L==NULL) return SoftI2C_Disable;
    if(i2c_opt->SDA_IN==NULL) return SoftI2C_Disable;
    if(i2c_opt->SDA_OUT==NULL) return SoftI2C_Disable;
    if(i2c_opt->SDA_R==NULL) return SoftI2C_Disable;
    if(i2c_opt->Delay_Us==NULL) return SoftI2C_Disable;
    return SoftI2C_Enable;
}

/**
 * @brief 软件I2C发送起始信号
 * @param i2c_opt (SoftI2C_Operate_t*)软i2c操作结构体指针
 * @return NULL
 * */
extern void SoftI2C_Start(SoftI2C_Operate_t* i2c_opt);

/**
 * @brief 软件I2C发送停止信号
 * @param i2c_opt (SoftI2C_Operate_t*)软i2c操作结构体指针
 * @return NULL
 * */
extern void SoftI2C_Stop(SoftI2C_Operate_t* i2c_opt);

/**
 * @brief 软件I2C接收应答信号
 * @param i2c_opt (SoftI2C_Operate_t*)软i2c操作结构体指针
 * @return 从机是否发送应答信号
 * */
extern SoftI2C_IsOK_t SoftI2C_WaitACK(SoftI2C_Operate_t* i2c_opt);

/**
 * @brief 软件I2C发送应答信号
 * @param i2c_opt (SoftI2C_Operate_t*)软i2c操作结构体指针
 * @return NULL
 * */
extern void SoftI2C_ACK(SoftI2C_Operate_t* i2c_opt);

/**
 * @brief 软件I2C发送非应答信号
 * @param i2c_opt (SoftI2C_Operate_t*)软i2c操作结构体指针
 * @return NULL
 * */
extern void SoftI2C_NACK(SoftI2C_Operate_t* i2c_opt);

/**
 * @brief 软件I2C发送1字节数据
 * @param i2c_opt (SoftI2C_Operate_t*) 软i2c操作结构体指针
 * @param txd (uint8_t) 发送的数据
 * @return NULL
 * */
extern void SoftI2C_Write_Byte(SoftI2C_Operate_t* i2c_opt, uint8_t txd);

/**
 * @brief 软件I2C接收1字节数据
 * @param i2c_opt (SoftI2C_Operate_t*)软i2c操作结构体指针
 * @return (uint8_t)接收到的数据
 * */
extern uint8_t SoftI2C_Read_Byte(SoftI2C_Operate_t* i2c_opt);

/**
 * @brief 软件I2C修改从机设备寄存器
 * @param i2c_opt (SoftI2C_Operate_t*)软i2c操作结构体
 * @param rw_mem (SoftI2C_RW_Mem_t*)软件I2C数据传输结构体指针
 * @return (uint8_t)是否传输成功
 * */
extern SoftI2C_IsOK_t SoftI2C_Write_Mem(SoftI2C_Operate_t* i2c_opt,SoftI2C_RW_Mem_t*rw_mem);

/**
 * @brief 软件I2C读取从机设备寄存器
 * @param i2c_opt (SoftI2C_Operate_t*)软i2c操作结构体
 * @param rw_mem (SoftI2C_RW_Mem_t*)软件I2C数据传输结构体指针
 * @return (uint8_t)是否传输成功
 * */
extern SoftI2C_IsOK_t SoftI2C_Read_Mem(SoftI2C_Operate_t* i2c_opt,SoftI2C_RW_Mem_t*rw_mem);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /*__BUS_SOFTI2C_H*/
