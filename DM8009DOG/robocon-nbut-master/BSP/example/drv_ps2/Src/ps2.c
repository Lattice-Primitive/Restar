#include "ps2.h"
#include "drv_ps2.h"
#include "main.h"
#include "drv_delay.h"

void ps2_cs_h(void)
{
    HAL_GPIO_WritePin(CS_GPIO_Port,CS_Pin,GPIO_PIN_SET);
}

void ps2_cs_l(void)
{
    HAL_GPIO_WritePin(CS_GPIO_Port,CS_Pin,GPIO_PIN_RESET);
}

void ps2_clk_h(void)
{
    HAL_GPIO_WritePin(CLK_GPIO_Port,CLK_Pin,GPIO_PIN_SET);
}

void ps2_clk_l(void)
{
    HAL_GPIO_WritePin(CLK_GPIO_Port,CLK_Pin,GPIO_PIN_RESET);
}

void ps2_do_h(void)
{
    HAL_GPIO_WritePin(DO_GPIO_Port,DO_Pin,GPIO_PIN_SET);
}

void ps2_do_l(void)
{
    HAL_GPIO_WritePin(DO_GPIO_Port,DO_Pin,GPIO_PIN_RESET);
}

uint32_t ps2_di_r(void)
{
    return HAL_GPIO_ReadPin(DI_GPIO_Port,DI_Pin);
}

PS2_Operation_t ps2_opt=
{
    .CS_H=ps2_cs_h,
    .CS_L=ps2_cs_l,
    .CLK_H=ps2_clk_h,
    .CLK_L=ps2_clk_l,
    .DO_H=ps2_do_h,
    .DO_L=ps2_do_l,
    .Delay_Us=Delay_Us,
    .DI_R=ps2_di_r,
};

PS2_Data_t ps2_data;

PS2_t ps2=
{
    .opt=&ps2_opt,
    .data=&ps2_data,
};

void PS2_Init(void)
{
    PS2_SetInit(&ps2);
}

void PS2_Run(void)
{
    PS2_Read(&ps2);
    HAL_Delay(20);
}
