/**
 * hwt906驱动程序
 * 接在板子上的I1,I2，暂时没接
 * */
#include "hwt906.h"
#include "pid.h"
#include "drv_hwt906.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c2;

void HWT906_WriteData_0(uint8_t dev_addr,uint8_t reg_addr,uint8_t *pdata,uint8_t data_size)
{
    HAL_I2C_Mem_Write(&hi2c2,dev_addr,reg_addr,I2C_MEMADD_SIZE_8BIT,pdata,data_size,1000);
}

void HWT906_ReadData_0(uint8_t dev_addr,uint8_t reg_addr,uint8_t *pdata,uint8_t data_size)
{
    HAL_I2C_Mem_Read(&hi2c2,dev_addr,reg_addr,I2C_MEMADD_SIZE_8BIT,pdata,data_size,1000);
}

HWT906_I2C_SPI_Operate_t wht906_opt1=
{
    HWT906_WriteData_0,
    HWT906_ReadData_0,
    HAL_Delay,
};

HWT906_I2C_SPI_t hwt906=
{
    0x50<<1,
    &wht906_opt1,
};

PID_t PID_IMU[3];

uint8_t rx_data[6];

/*配置HWT906陀螺仪并配置PID*/
void HWT906_Init(void)
{
    PID_Set_t pid_set=
    {
        .pid_mode=POSITION_PID,
        .MaxOutput=10000,
        .IntegralLimit=0,
        .p=1.5f,
        .i=0.01,
        .d=0,
    };
    PID_Init(&PID_IMU[0],&pid_set);
    PID_Init(&PID_IMU[1],&pid_set);
    PID_IMU[2].p=1.30f;
    PID_IMU[2].i=0;
    PID_Init(&PID_IMU[2],&pid_set);
    HWT906_I2C_SPI_Reboot(&hwt906);
		
}

/*HWT906陀螺仪读取角度数据*/
void HWT906_Read(AHRS_Angle_t *Now_Angle) 
{
    HAL_I2C_Mem_Read(&hi2c2,hwt906.addr,HWT906_ANGLE_X,I2C_MEMADD_SIZE_8BIT,rx_data,6,1000);
    Now_Angle->Roll=((int16_t)((uint16_t)rx_data[1]<<8|rx_data[0]))*180.0f/32768.0f;
    Now_Angle->Pitch=((int16_t)((uint16_t)rx_data[3]<<8|rx_data[2]))*180.0f/32768.0f;
    Now_Angle->Yaw=((int16_t)((uint16_t)rx_data[5]<<8|rx_data[4]))*180.0f/32768.0f;
}
