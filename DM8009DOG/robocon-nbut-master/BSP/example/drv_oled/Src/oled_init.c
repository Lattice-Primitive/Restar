#include "oled_init.h"
#include "main.h"
#include "drv_delay.h"
#include "bus_softi2c.h"
#include "drv_oled.h"

static uint8_t gram_buffer[128][8];

uint8_t OLED0_Send_DataByte(uint8_t data);
uint8_t OLED0_Send_CommandByte(uint8_t cmd);

GPIO_InitTypeDef OLED_GPIO_Init=
{
  .Pin=GPIO_PIN_5,
  .Mode = GPIO_MODE_OUTPUT_OD,
  .Pull = GPIO_NOPULL,
  .Speed = GPIO_SPEED_FREQ_HIGH,
};

void OLED_Delay_Us(uint32_t ticks)
{
    Delay_Us(ticks);
}

void OLED_SCL_H(void)
{
  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_SET);
}

void OLED_SCL_L(void)
{
  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_RESET);
}

void OLED_SDA_H(void)
{
  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_SET);
}

void OLED_SDA_L(void)
{
  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_RESET);
}

void OLED_SDA_IN(void)
{
  OLED_GPIO_Init.Mode=GPIO_MODE_INPUT;
  HAL_GPIO_Init(GPIOE, &OLED_GPIO_Init);
}

void OLED_SDA_OUT(void)
{
  OLED_GPIO_Init.Mode=GPIO_MODE_OUTPUT_OD;
  HAL_GPIO_Init(GPIOE, &OLED_GPIO_Init);
}

uint32_t OLED_SDA_R(void)
{
  return HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_5);
}

SoftI2C_Operate_t i2copts=
{
  .Delay_Us=OLED_Delay_Us,
  .SCL_H=OLED_SCL_H,
  .SCL_L=OLED_SCL_L,
  .SDA_H=OLED_SDA_H,
  .SDA_L=OLED_SDA_L,
  .SDA_IN=OLED_SDA_IN,
  .SDA_OUT=OLED_SDA_OUT,
  .SDA_R=OLED_SDA_R,
};

OLED_Operate_t oled0_opts=
{
    .OLED_Send_DataByte=OLED0_Send_DataByte,
    .OLED_Send_CommandByte=OLED0_Send_CommandByte,
};

OLED_t oled0=
{
    (uint32_t)gram_buffer,
    64,
    128,
    &oled0_opts,
    0x3c
};


uint8_t OLED0_Send_DataByte(uint8_t data)
{
    SoftI2C_Start(&i2copts);
    SoftI2C_Write_Byte(&i2copts, (oled0.addr) << 1);
    if (SoftI2C_NOK == SoftI2C_WaitACK(&i2copts))
    {
        return SoftI2C_NOK;
    }
    SoftI2C_Write_Byte(&i2copts, 0xc0);
    if (SoftI2C_NOK == SoftI2C_WaitACK(&i2copts))
    {
        return SoftI2C_NOK;
    }
    SoftI2C_Write_Byte(&i2copts, data);
    if (SoftI2C_NOK == SoftI2C_WaitACK(&i2copts))
    {
        return SoftI2C_NOK;
    }
    SoftI2C_Stop(&i2copts);
    return SoftI2C_OK;

}

uint8_t OLED0_Send_CommandByte(uint8_t cmd)
{
    
    SoftI2C_Start(&i2copts);
    SoftI2C_Write_Byte(&i2copts, (oled0.addr) << 1);
    if (SoftI2C_NOK == SoftI2C_WaitACK(&i2copts))
    {
        return SoftI2C_NOK;
    }
    SoftI2C_Write_Byte(&i2copts, 0x80);
    if (SoftI2C_NOK == SoftI2C_WaitACK(&i2copts))
    {
        return SoftI2C_NOK;
    }
    SoftI2C_Write_Byte(&i2copts, cmd);
    if (SoftI2C_NOK == SoftI2C_WaitACK(&i2copts))
    {
        return SoftI2C_NOK;
    }
    SoftI2C_Stop(&i2copts);
    return SoftI2C_OK;
}

void OLED0_Init(void)
{
    OLED_Init(&oled0);
    OLED_Display_On(&oled0);
}

void OLED0_Display(void)
{
		OLED_Gram_Clear(&oled0);
    OLED_Gram_DrawString(&oled0,0,0,"ok?",3,ASCII_1206,1);
    OLED_Gram_refresh(&oled0);
    HAL_Delay(500);
		OLED_Gram_Clear(&oled0);
		OLED_Gram_DrawString(&oled0,0,0,"not ok!",7,ASCII_1206,1);
    OLED_Gram_refresh(&oled0);
		HAL_Delay(500);
    //OLED_Gram_Clear(&oled0);
    //HAL_Delay(50);
}
