#include "drv_hwt906.h"
#include "main.h"
#include "stm32f4xx_hal_i2c.h"

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

int16_t Yaw_Angle;

void HWT906_Init(void)
{
    HWT906_I2C_SPI_Reboot(&hwt906);
    HWT906_I2C_SPI_Correction(&hwt906,1000);
}

void HWT906_Sampling(void)
{
    Yaw_Angle=HWT906_I2C_SPI_ReadOrderData(&hwt906,HWT906_ANGLE_Z)*180U/32768U;
    HAL_Delay(100);
}
