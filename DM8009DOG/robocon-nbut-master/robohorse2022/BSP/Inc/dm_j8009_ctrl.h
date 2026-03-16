#ifndef DM_J8009_CTRL_H
#define DM_J8009_CTRL_H


#include "main.h"
#include "gait_param.h"
#include "pid.h"
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



/**
 * @brief M3508输出电流结构体
 */
typedef struct
{
    int16_t ID1;
    int16_t ID2;
    int16_t ID3;
    int16_t ID4;
    int16_t ID5;
    int16_t ID6;
    int16_t ID7;
    int16_t ID8;
} DM8009pos_t;


//达妙电机位置环控制
void dm_j8009_pos_ctrl(CAN_HandleTypeDef can,uint8_t id, float set_pos, float kp, float kd);

/**
  * @brief          达妙电机控制
  * @retval         none
  */
void dm_j8009_all_ctrl(float SET_Angle[8],PID_t PID_SET[8]);


void dm_PD_set(PID_t *pid, LegGain state_pid);

void dm_j8009_all_ctrl(float SET_Angle[8],PID_t PID_SET[8]);

void dm_j8009_all_enable();

void dm_j8009_all_disable();







#endif





