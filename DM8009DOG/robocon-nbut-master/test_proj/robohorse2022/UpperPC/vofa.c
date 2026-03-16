/**
  ****************************(C) COPYRIGHT 2024 Tan****************************
  * @file       vafa.c/h
  * @brief     这是与VOFA上位机配合使用的文件，可以将数据输出用上位机观测.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *	 V2.0.0			2024-5-14			R&A战队					1. done
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Tan****************************
  */


#include "vofa.h"
#include "usart.h"
#include "string.h"



#define TX_BUFF_MAX	256				//缓存数组长度

uint8_t VOFA_TX_BUFF[TX_BUFF_MAX];			//发送缓存数组
uint16_t len;








void float_to_buff(float in){
	memcpy(VOFA_TX_BUFF+len,(uint8_t*)&in,4);
	len += 4;
}




void vofa_task(void){
	len = 0;
	
	
	
//	float_to_buff(*chassis_move.chassis_INS_angle_Z/PI*180.0f);		//旋转实时角度
//	float_to_buff(chassis_move.set_turn_angle);										//旋转目标角度
//	
//	float_to_buff(chassis_move.chassis_motor_data[1]->motor.enginer_motor_measure->speed_rpm);		//实时速度
//	float_to_buff(chassis_move.chassis_motor_data[1]->motor.target_speed);										//目标速度
//	
//	float_to_buff(*chassis_move.chassis_INS_ACCEL_X);
//	
//    float_to_buff(*chassis_move.chassis_INS_ACCEL_X);//dm1 实时角度
//	float_to_buff(*chassis_move.chassis_INS_ACCEL_X);//dm1 目标角度
    
	//VOFA+上位机指定的数据接收后缀
	VOFA_TX_BUFF[len++]=0x00;
	VOFA_TX_BUFF[len++]=0x00;
	VOFA_TX_BUFF[len++]=0x80;
	VOFA_TX_BUFF[len++]=0x7f;
	
	HAL_UART_Transmit_DMA(&huart1,VOFA_TX_BUFF,len);
}

























