#include "main.h"
#include "cmsis_os.h"
#include "enginer_task.h"
#include "CAN_receive.h"
#include "can.h"
#include "remote_control.h"
#include "power_limit_pid.h"

#include "dm_j8009_ctrl.h"
#include "dm8009_drv.h"

fp32 M3508_SPEED_pid[3]={M3508_MOTOR_SPEED_PID_KP,M3508_MOTOR_SPEED_PID_KI,M3508_MOTOR_SPEED_PID_KD};    //M3508电机速度环pid
fp32 M3508_POSITION_pid[3]={M3508_MOTOR_POSITION_PID_KP,M3508_MOTOR_POSITION_PID_KI,M3508_MOTOR_POSITION_PID_KD};    //M3508电机位置环pid

fp32 M2006_SPEED_pid[3] = {M2006_MOTOR_SPEED_PID_KP,M2006_MOTOR_SPEED_PID_KI,M2006_MOTOR_SPEED_PID_KD};
fp32 M2006_POSITION_pid[3] = {M2006_MOTOR_POSITION_PID_KP,M2006_MOTOR_POSITION_PID_KI,M2006_MOTOR_POSITION_PID_KD};
/**
	CAN1 0 1 2 3 为底盘电机 6 7 是一级抬升电机
			 4 x轴移动电机 5 传送带电机
	
	CAN2 0 1 2 机械臂小roll 大roll 小p 
	CAN2 4 伸出臂
	CAN2 3 大P
	CAN2 5 6 二级抬升
  */
/**
  * @brief          电机初始化函数
  * @param[in]      none
  * @retval         none
  */
void Motor_Init(){
	//初始化0`4 6`7号电机 CAN1 为3508电机
	for(uint8_t i = 0; i < 8; i++){
		if((i < 4) || (i >= 6 && i < 8))
		{
			enginer_init(&enginer_move_can1[i],i,1,M3508_SPEED_pid,M3508_MOTOR_SPEED_PID_MAX_OUT,M3508_MOTOR_SPEED_PID_MAX_IOUT, \
			M3508_POSITION_pid,M3508_MOTOR_POSITION_PID_MAX_OUT,M3508_MOTOR_POSITION_PID_MAX_IOUT);
		}
		else// 4`5 2006
		{
			enginer_init(&enginer_move_can1[i],i,1,M2006_SPEED_pid,M2006_MOTOR_SPEED_PID_MAX_OUT,M2006_MOTOR_SPEED_PID_MAX_IOUT,\
			M2006_POSITION_pid,M2006_MOTOR_POSITION_PID_MAX_OUT,M2006_MOTOR_POSITION_PID_MAX_IOUT);	
		}
	}
	
	for(uint8_t i = 0; i < 7; i++){
		if((i<3) || i==4) // 1235 2006
		{
			enginer_init(&enginer_move_can2[i],i,2,M2006_SPEED_pid,M2006_MOTOR_SPEED_PID_MAX_OUT,M2006_MOTOR_SPEED_PID_MAX_IOUT,\
			M2006_POSITION_pid,M2006_MOTOR_POSITION_PID_MAX_OUT,M2006_MOTOR_POSITION_PID_MAX_IOUT);
		}
		else// 4 67 3508
		{
			enginer_init(&enginer_move_can2[i],i,2,M3508_SPEED_pid,M3508_MOTOR_SPEED_PID_MAX_OUT,M3508_MOTOR_SPEED_PID_MAX_IOUT, \
			M3508_POSITION_pid,M3508_MOTOR_POSITION_PID_MAX_OUT,M3508_MOTOR_POSITION_PID_MAX_IOUT);
		}
	}
}



void Motor_Task(void const * argument){
	//电机的角度、速度PID初始化
	Motor_Init();
	osDelay(5000);    //等待5s，使系统完全运行后才能进入电机任务
	while(1){
		//对CAN1各个电机角度环、速度环进行PID解算
		for(uint8_t i = 0; i < 8; i++){
			if((i < 4) || (i >= 6 && i < 8))
			{
				PIDrealize(&enginer_move_can1[i],SPEED_RING_CONTROL,M3508_MOTOR_ECD_TO_ANGLE);
			}
			else    //4 5 2006
			{
				PIDrealize(&enginer_move_can1[i],SPEED_RING_CONTROL,M2006_MOTOR_ECD_TO_ANGLE);
			}
		}		
		//对CAN2各个电机角度环、速度环进行PID解算
		for(uint8_t i = 0; i < 7; i++){
		if((i<3) || i==4) // 1235 2006
			{
				PIDrealize(&enginer_move_can2[i],SPEED_RING_CONTROL,M2006_MOTOR_ECD_TO_ANGLE);
			}
			else // 4 67 3508
			{
				PIDrealize(&enginer_move_can2[i],SPEED_RING_CONTROL,M3508_MOTOR_ECD_TO_ANGLE);
			}
	}
		//功率限制
		Chassis_Power_Cal();
		
		if(!switch_is_down(rc_ctrl.rc.s[0])){
			while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan1));     		//CAN1邮箱为0时等待发送
			//CAN1发送ID0~3的电机电流数据
			CAN1_cmd_0_3(enginer_move_can1[0].motor.give_current,enginer_move_can1[1].motor.give_current,enginer_move_can1[2].motor.give_current,enginer_move_can1[3].motor.give_current);
			
			while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan1));     		//CAN1邮箱为0时等待发送
			//CAN1发送ID4~7的电机电流数据
			CAN1_cmd_4_7(enginer_move_can1[4].motor.give_current,enginer_move_can1[5].motor.give_current,enginer_move_can1[6].motor.give_current,enginer_move_can1[7].motor.give_current);
			
			while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan2));     		//CAN2邮箱为0时等待发送
			//CAN2发送ID0~3的电机电流数据
			CAN2_cmd_0_3(enginer_move_can2[0].motor.give_current,enginer_move_can2[1].motor.give_current,enginer_move_can2[2].motor.give_current,enginer_move_can2[3].motor.give_current);
			
			while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan2));     		//CAN2邮箱为0时等待发送
			//CAN2发送ID4~7的电机电流数据
			CAN2_cmd_4_7(enginer_move_can2[4].motor.give_current,enginer_move_can2[5].motor.give_current,enginer_move_can2[6].motor.give_current,enginer_move_can2[7].motor.give_current);
		}
		else{
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
		}
		osDelay(10);    //延迟10ms
	}
}



