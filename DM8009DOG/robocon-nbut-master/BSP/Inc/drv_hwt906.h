/**
 * @file drv_hwt906.h HWT906陀螺仪驱动
 * @author zhiyuan Mao(2019th) (全平台)
 * @brief 用于全平台的HWT906陀螺仪驱动，支持I2C和SPI，后续支持UART
*/
#ifndef __DRV_HWT906_H
#define __DRV_HWT906_H

#ifdef __cplusplus
extern "C"
{
#endif /*__cplusplus*/

#include <stdint.h>
#include <stddef.h>

typedef enum 
{
    HWT906_ACCEL_X=0x34, //x轴加速度
    HWT906_ACCEL_Y=0x35, //y轴加速度
    HWT906_ACCEL_Z=0x36, //z轴加速度

    HWT906_GYRO_X=0x37, //x轴角速度
    HWT906_GYRO_Y=0x38, //y轴角速度
    HWT906_GYRO_Z=0x39, //z轴角速度

    HWT906_MAGNET_X=0x3a, //x轴磁力
    HWT906_MAGNET_Y=0x3b, //y轴磁力
    HWT906_MAGNET_Z=0x3c, //z轴磁力

    HWT906_ANGLE_X=0x3d, //x轴角度
    HWT906_ANGLE_Y=0x3e, //y轴角度
    HWT906_ANGLE_Z=0x3f, //z轴角度

    HWT906_TEMP=0x40, //温度
    
    HWT906_QUAT_Q0=0x51, //四元数0
    HWT906_QUAT_Q1=0x52, //四元数1
    HWT906_QUAT_Q2=0x53, //四元数2
    HWT906_QUAT_Q3=0x54, //四元数3
}HWT906_DataType_t;

typedef struct
{
    /* @brief HWT906写入寄存器 
     * @param dev_addr 器件地址 
     * @param reg_addr 寄存器地址 
     * @param pdata 写入数据指针 
     * @param data_size 数据大小
     * @return (void) NULL*/
    void (*HWT906_WriteData)(uint8_t dev_addr,uint8_t reg_addr,uint8_t *pdata,uint8_t data_size); 
    /* @brief HWT906读取寄存器 
     * @param dev_addr 器件地址
     * @param reg_addr 寄存器地址 
     * @param pdata 读出数据指针 
     * @param data_size 数据大小
     * @return (void) NULL*/
    void (*HWT906_ReadData)(uint8_t dev_addr,uint8_t reg_addr,uint8_t *pdata,uint8_t data_size); 
    /* @brief HWT906陀螺仪延时 
     * @param ticks 延时时间
     * @return (void) NULL*/
    void (*HWT906_Delay_Ms)(uint32_t ticks); 
}HWT906_I2C_SPI_Operate_t;

typedef struct
{
    uint8_t addr; //HWT906器件地址
    HWT906_I2C_SPI_Operate_t* opt; //HWT906操作结构体函数
}HWT906_I2C_SPI_t;

typedef struct 
{
    int16_t x; //x轴数据
    int16_t y; //y轴数据
    int16_t z; //z轴数据
}HWT906_Data_t;

typedef struct
{
    HWT906_Data_t *accel; //加速度
    HWT906_Data_t *gyro; //角速度
    HWT906_Data_t *magnet; //磁力计
    HWT906_Data_t *angle; //角度
    int16_t temp; //温度
    int16_t quat[4]; //四元数
}HWT906_DataPack_t;

/**
 * @brief HWT906陀螺仪重启
 * @param hwt906 (HWT906_I2C_SPI_t*) HWT906陀螺仪结构体
 * @return (void) NULL
*/
extern void HWT906_I2C_SPI_Reboot(HWT906_I2C_SPI_t* hwt906);

/**
 * @brief HWT906陀螺仪校正
 * @param hwt906 (HWT906_I2C_SPI_t*) HWT906陀螺仪结构体
 * @param ticks (uint32_t) 延时时间
 * @return (void) NULL
*/
extern void HWT906_I2C_SPI_Correction(HWT906_I2C_SPI_t* hwt906,uint32_t ticks);

/**
 * @brief HWT906陀螺仪读取所有数据
 * @param hwt906 (HWT906_I2C_SPI_t*) HWT906陀螺仪结构体
 * @param pack (HWT906_DataPack_t*) HWT906陀螺仪数据包结构体
 * @return (void) NULL
*/
extern void HWT906_I2C_SPI_ReadAllData(HWT906_I2C_SPI_t* hwt906,HWT906_DataPack_t* pack);

/**
 * @brief HWT906陀螺仪读取单个数据
 * @param hwt906 (HWT906_I2C_SPI_t*) HWT906陀螺仪结构体
 * @param data_type (HWT906_DataType_t) HWT906陀螺仪数据类型
 * @return (int16_t) 读取到的原始数据
*/
extern int16_t HWT906_I2C_SPI_ReadOrderData(HWT906_I2C_SPI_t* hwt906,HWT906_DataType_t data_type);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /*__DRV_HWT906_H*/
