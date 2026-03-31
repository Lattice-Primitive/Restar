#include "bsp_pump.h"
#include "cmsis_os.h"
#include "main.h" 

_Bool Sov_flag;
void pump_on(void)
{
	HAL_GPIO_WritePin(Powerful_03_GPIO_Port,Powerful_03_Pin,GPIO_PIN_SET);
	Sov_flag = 1;
}

void pump_off(void)
{
	HAL_GPIO_WritePin(Powerful_03_GPIO_Port,Powerful_03_Pin,GPIO_PIN_RESET);
	if(Sov_flag){
		HAL_GPIO_WritePin(Powerful_04_GPIO_Port,Powerful_04_Pin,GPIO_PIN_SET);
		osDelay(200);
		HAL_GPIO_WritePin(Powerful_04_GPIO_Port,Powerful_04_Pin,GPIO_PIN_RESET);
	}
	Sov_flag = 0;
}










