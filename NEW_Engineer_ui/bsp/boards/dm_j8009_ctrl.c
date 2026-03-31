#include "dm_j8009_ctrl.h"
#include "dm8009_drv.h"
#include "can.h"

//关节电机位置限幅
float pos_limit_min[8] = {MOTOR_0_POS_MIN, MOTOR_1_POS_MIN, MOTOR_2_POS_MIN, MOTOR_3_POS_MIN,MOTOR_4_POS_MIN,MOTOR_5_POS_MIN, MOTOR_6_POS_MIN, MOTOR_7_POS_MIN};
float pos_limit_max[8] = {MOTOR_0_POS_MAX, MOTOR_1_POS_MAX, MOTOR_2_POS_MAX, MOTOR_3_POS_MAX,MOTOR_4_POS_MAX,MOTOR_5_POS_MAX,MOTOR_6_POS_MAX,MOTOR_7_POS_MAX,};



/**
  * @brief          达妙电机位置环控制
  * @param[in]      id: 电机id
  * @param[in]      set_pos: 目标位置
  * @param[in]      kp: 控制参数KP
  * @param[in]     	kd: 控制参数KD
  * @retval         none
  */
void dm_j8009_pos_ctrl(CAN_HandleTypeDef can,uint8_t id, float set_pos, float kp, float kd){
//	if(id > 7)			//关节电机ID为1~4
//		return;
	//关节先限幅   防止超调卡住损坏电机
	if(set_pos > pos_limit_max[id]){				//先是最大值限幅
		set_pos = pos_limit_max[id];
	}
	if(set_pos < pos_limit_min[id]){				//再是最小值限幅
		set_pos = pos_limit_min[id];
	}
	
	CTRL_DM_J8009_MIT(can,id,set_pos,0,kp,kd,0);
	
}














