#include "dm_j8009_ctrl.h"
#include "dm8009_drv.h"

#include "can.h"
#include "pid.h"

//关节电机位置限幅
float pos_limit_min[8] = {MOTOR_0_POS_MIN, MOTOR_1_POS_MIN, MOTOR_2_POS_MIN, MOTOR_3_POS_MIN,MOTOR_4_POS_MIN,MOTOR_5_POS_MIN, MOTOR_6_POS_MIN, MOTOR_7_POS_MIN};
float pos_limit_max[8] = {MOTOR_0_POS_MAX, MOTOR_1_POS_MAX, MOTOR_2_POS_MAX, MOTOR_3_POS_MAX,MOTOR_4_POS_MAX,MOTOR_5_POS_MAX,MOTOR_6_POS_MAX,MOTOR_7_POS_MAX,};

extern uint8_t DM_J8009_ENABLE_FLAG;
extern uint8_t DM_J8009_ctrl_FLAG;

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


void dm_PD_set(PID_t *pid, LegGain state_pid)
{
    pid->p = state_pid.kp_pos;
    pid->d = state_pid.kd_pos;
}


/**
  * @brief          达妙电机控制
  * @retval         none
  */
void dm_j8009_all_ctrl(float SET_Angle[8],PID_t PID_SET[8]){
		
			for(uint8_t id=0;id<=0x07;id++)
			{
				if(id==0x00||id==0x01||id==0x06||id==0x07){
						while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan1));     		//CAN2邮箱为0时等待发送
						dm_j8009_pos_ctrl(hcan1,id,SET_Angle[id], PID_SET[id].p, PID_SET[id].d);
						vTaskDelay(1);    //延迟1ms
					}
					else if(id==0x02||id==0x03||id==0x04||id==0x05){
						while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan2));     		//CAN2邮箱为0时等待发送
						dm_j8009_pos_ctrl(hcan2,id,SET_Angle[id], PID_SET[id].p, PID_SET[id].d);
						vTaskDelay(1);    //延迟1ms
					}
					DM_J8009_ctrl_FLAG = 1;  
			}
				
	
}

/**
  * @brief          达妙电机使能
  * @retval         none
  */
void dm_j8009_all_enable(){
			while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan1));     		//CAN2邮箱为0时等待发送
			ENABLE_DM_J8009(hcan1,0x00);
			vTaskDelay(1);    //延迟1ms
			while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan1));     		//CAN2邮箱为0时等待发送
			ENABLE_DM_J8009(hcan1,0x01);
			vTaskDelay(1);    //延迟1ms
			while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan1));     		//CAN2邮箱为0时等待发送
			ENABLE_DM_J8009(hcan1,0x06);
			vTaskDelay(1);    //延迟1ms
			while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan1));     		//CAN2邮箱为0时等待发送
			ENABLE_DM_J8009(hcan1,0x07);
			vTaskDelay(1);    //延迟1ms
			while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan2));     		//CAN2邮箱为0时等待发送
			ENABLE_DM_J8009(hcan2,0x02);
			vTaskDelay(1);    //延迟1ms
			while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan2));     		//CAN2邮箱为0时等待发送
			ENABLE_DM_J8009(hcan2,0x03);
			vTaskDelay(1);    //延迟1ms
			while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan2));     		//CAN2邮箱为0时等待发送
			ENABLE_DM_J8009(hcan2,0x04);
			vTaskDelay(1);    //延迟1ms
			while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan2));     		//CAN2邮箱为0时等待发送
			ENABLE_DM_J8009(hcan2,0x05);
			vTaskDelay(1);    //延迟1ms
			DM_J8009_ENABLE_FLAG = 1;
	
}


/**
  * @brief          达妙电机失能
  * @retval         none
  */
void dm_j8009_all_disable(){
			while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan1));     		//CAN2邮箱为0时等待发送
			DISABLE_DM_J8009(hcan1,0x00);
			vTaskDelay(1);    //延迟1ms
			while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan1));     		//CAN2邮箱为0时等待发送
			DISABLE_DM_J8009(hcan1,0x01);
			vTaskDelay(1);    //延迟1ms
			while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan1));     		//CAN2邮箱为0时等待发送
			DISABLE_DM_J8009(hcan1,0x06);
			vTaskDelay(1);    //延迟1ms
			while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan1));     		//CAN2邮箱为0时等待发送
			DISABLE_DM_J8009(hcan1,0x07);
			vTaskDelay(1);    //延迟1ms
			while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan2));     		//CAN2邮箱为0时等待发送
			DISABLE_DM_J8009(hcan2,0x02);
			vTaskDelay(1);    //延迟1ms
			while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan2));     		//CAN2邮箱为0时等待发送
			DISABLE_DM_J8009(hcan2,0x03);
			vTaskDelay(1);    //延迟1ms
			while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan2));     		//CAN2邮箱为0时等待发送
			DISABLE_DM_J8009(hcan2,0x04);
			vTaskDelay(1);    //延迟1ms
			while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan2));     		//CAN2邮箱为0时等待发送
			DISABLE_DM_J8009(hcan2,0x05);
			vTaskDelay(1);    //延迟1ms
			DM_J8009_ENABLE_FLAG = 0;			
	
}














