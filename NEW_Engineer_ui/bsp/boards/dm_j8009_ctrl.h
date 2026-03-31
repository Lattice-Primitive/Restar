#ifndef DM_J8009_CTRL_H
#define DM_J8009_CTRL_H


#include "main.h"


//各个关节电机的机械位置最大最小角度
#define 		MOTOR_0_POS_MIN			-1.6f				
#define 		MOTOR_0_POS_MAX			1.6f
#define 		MOTOR_1_POS_MIN			-1.6f				
#define 		MOTOR_1_POS_MAX			1.6f
#define 		MOTOR_2_POS_MIN			-1.6f				
#define 		MOTOR_2_POS_MAX			1.6f
#define 		MOTOR_3_POS_MIN			-1.6f				
#define 		MOTOR_3_POS_MAX			1.6f
#define 		MOTOR_4_POS_MIN			-1.6f	
#define 		MOTOR_4_POS_MAX			1.6f
#define 		MOTOR_5_POS_MIN			-1.6f				
#define 		MOTOR_5_POS_MAX			1.6f
#define 		MOTOR_6_POS_MIN			-1.6f				
#define 		MOTOR_6_POS_MAX			1.6f
#define 		MOTOR_7_POS_MIN			-1.6f				
#define 		MOTOR_7_POS_MAX			1.6f


















//达妙电机位置环控制
void dm_j8009_pos_ctrl(CAN_HandleTypeDef can,uint8_t id, float set_pos, float kp, float kd);















#endif





