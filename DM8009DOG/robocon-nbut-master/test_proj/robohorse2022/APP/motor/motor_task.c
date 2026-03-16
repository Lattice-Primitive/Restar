#include <stdarg.h>
#include <stdint.h>

#include "motor_task.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"


#include "pid.h"
#include "gait_param.h"
#include "usart.h"
#include "app_ebf_piddebug.h"
#include "standup_task.h"

#include "dm_j8009_ctrl.h"
#include "dm8009_drv.h"
#include "CAN_receive.h"



PID_t PID_Position[8];  //位置PID
PID_t PID_Speed[8];
PID_t PID_IMU_Climbing; //爬坡用PID
PID_t PID_IMU_Line;     //直线修正pid


extern float imu_Leg_offset;     //陀螺仪偏移量
float PID_Aim_Angle[8]; //将用于PID运算的目标角度
DM8009pos_t Motor_Output; //位置
#define PID_CONFIG 0

#if PID_CONFIG == 0

void Motor_Init(){
	
    // 修改电机ID映射关系
    // CAN1: ID1-4 -> 右前腿(0,1), 右后腿(2,3)
    // CAN2: ID5-8 -> 左前腿(4,5), 左后腿(6,7)
    
   

    for (uint8_t i = 0; i < 4; i++)
    {
        Stanford_Type_Lite_Init(&Gait_Data[i], 10.0f, 20.0f); //设置腿部长度参数
    }
	osDelay(500);
	/***********************************************达妙关节电机使能****************************************************************/
//	while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan2));     		//CAN2邮箱为0时等待发送
//	ENABLE_DM_J8009(0x01);
//	vTaskDelay(1);    //延迟1ms
//	while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan2));     		//CAN2邮箱为0时等待发送
//	ENABLE_DM_J8009(0x02);
//	vTaskDelay(1);    //延迟1ms
//	while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan2));     		//CAN2邮箱为0时等待发送
//	ENABLE_DM_J8009(0x03);
//	vTaskDelay(1);    //延迟1ms
//	while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan2));     		//CAN2邮箱为0时等待发送
//	ENABLE_DM_J8009(0x04);
//	vTaskDelay(1);    //延迟1ms
	/*********************************************************************************************************************/
	
/***********************************************8009保存零点****************************************************************/
//	while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan1));     		//CAN2邮箱为0时等待发送
//	SAVE_ZERO_DM_J8009(hcan1,0x00);
//	vTaskDelay(1);   
//	while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan1));     		//CAN2邮箱为0时等待发送
//	SAVE_ZERO_DM_J8009(hcan1,0x01);
//	vTaskDelay(1);   
//	while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan2));     		//CAN2CAN2邮箱为0时等待发送
//	SAVE_ZERO_DM_J8009(hcan2,0x02);
//	vTaskDelay(1);   
//	while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan2));     		//CAN2CAN2邮箱为0时等待发送
//	SAVE_ZERO_DM_J8009(hcan2,0x03);
//	vTaskDelay(1);    
//	while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan2));     		//CAN2CAN2邮箱为0时等待发送
//	SAVE_ZERO_DM_J8009(hcan2,0x04);
//	vTaskDelay(1);    
//	while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan2));     		//CAN2CAN2邮箱为0时等待发送
//	SAVE_ZERO_DM_J8009(hcan2,0x05);
////	vTaskDelay(1);   
//	while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan1));     		//CAN2CAN2邮箱为0时等待发送
//	SAVE_ZERO_DM_J8009(hcan1,0x06);
//	vTaskDelay(1);    
//	while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan1));     		//CAN2CAN2邮箱为0时等待发送
//	SAVE_ZERO_DM_J8009(hcan1,0x07);
//	vTaskDelay(1);   
	/*********************************************************************************************************************/}


uint8_t DM_J8009_ENABLE_FLAG = 1;				//???????ú8009????±ê??  ??0?±??????  ??1?±????
float angle;
/**
 * 电机驱动任务
 * 流程：装载新的位置-> 双环PID运算->发送新的电流值
 * */
void Motor_Task(void *argument)
{
    while (1)
    {
        if (Is_Aim_Angle_Get == 1) //是否已经获得了新的目标角度
        {
            for (uint8_t i = 0; i < 8; i++)
            {
                // 修改电机ID映射关系
                // 右前腿(0): 电机0,1
                // 右后腿(1): 电机2,3
                // 左前腿(2): 电机4,5
                // 左后腿(3): 电机6,7
                if (i == 0 || i == 1) { // 右前腿
                    PID_Aim_Angle[i] = Aim_Angle[i] + Standup_LegR_Offset - imu_Leg_offset;
                }
                else if (i == 2 || i == 3) { // 右后腿
                    PID_Aim_Angle[i] = Aim_Angle[i] + Standup_LegR_Offset - imu_Leg_offset;
                }
                else if (i == 4 || i == 5) { // 左前腿
                    PID_Aim_Angle[i] = Aim_Angle[i] + Standup_LegL_Offset + imu_Leg_offset;
                }
                else if (i == 6 || i == 7) { // 左后腿
                    PID_Aim_Angle[i] = Aim_Angle[i] + Standup_LegL_Offset + imu_Leg_offset;
                }
            }
            Is_Aim_Angle_Get = 0; //清零标志
        }

       
        Motor_Output.ID1 = PID_Aim_Angle[0];
        Motor_Output.ID2 = PID_Aim_Angle[1];
        Motor_Output.ID3 = PID_Aim_Angle[2];
        Motor_Output.ID4 = PID_Aim_Angle[3];
        Motor_Output.ID5 = PID_Aim_Angle[4];
        Motor_Output.ID6 = PID_Aim_Angle[5];
        Motor_Output.ID7 = PID_Aim_Angle[6];
        Motor_Output.ID8 = PID_Aim_Angle[7];
        

        /* 绝对延时 */
        static portTickType xLastWakeTime;
        static const portTickType xFrequency = pdMS_TO_TICKS(3);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        //        osDelay(3);
    }
}

void Motor_Set_MaxSpeed(PID_t *pid, uint16_t speed) //设置最大速度      
{
    for (uint8_t i = 0; i < 8; i++)
    {
        pid[i].MaxOutput = speed;
    }
}
#else

static uint32_t Delay_Time = 5;

void _Set_Period(uint32_t period)    //设置周期
{
    Delay_Time = period;
}

void _Transmit(uint8_t *pdata, uint32_t data_size) //发送数据
{
    HAL_UART_Transmit(&huart7, pdata, data_size, 100);
}

void _SetPID(float p, float i, float d) //设置PID       
{
    int now_target = EBFPIDDebuger_Get_Target_Value(); //目标值用为设置PID的目标
    if (now_target >= 0 && now_target < 8)
    {
        PID_Position[now_target].p = p;
        PID_Position[now_target].i = i;
        PID_Position[now_target].d = d;
    }
}

EBFPIDDebuger_Opts pid_config_opts =
    {
        .Set_Period = _Set_Period,
        .Transmit = _Transmit,
        .SetPID = _SetPID,
};

/**
 * 电机驱动任务
 * 流程：装载新的位置-> 双环PID运算->发送新的电流值
 * */
void Motor_Task(void *argument)
{
    while (1)
    {
        if (Is_Aim_Angle_Get == 1) //是否已经获得了新的目标角度
        {
            for (uint8_t i = 0; i < 8; i++)
            {
                PID_Aim_Angle[i] = Aim_Angle[i]; //数据转移
            }
            Is_Aim_Angle_Get = 0; //清零标志
        }
        for (uint8_t i = 0; i < 8; i++) //双环PID运算
        {
            // 位置PID
            PID_Calc(&PID_Position[i], M3508[i].Total_Angle * 360 / 8191, PID_Aim_Angle[i] * ReductionAndAngleRatio);
            // 速度PID
            PID_Calc(&PID_Speed[i], M3508[i].Speed_RPM, PID_Position[i].pos_out);
        }
        Motor_Output.ID1 = PID_Speed[0].pos_out;
        Motor_Output.ID2 = PID_Speed[1].pos_out;
        Motor_Output.ID3 = PID_Speed[2].pos_out;
        Motor_Output.ID4 = PID_Speed[3].pos_out;
        Motor_Output.ID5 = PID_Speed[4].pos_out;
        Motor_Output.ID6 = PID_Speed[5].pos_out;
        Motor_Output.ID7 = PID_Speed[6].pos_out;
        Motor_Output.ID8 = PID_Speed[7].pos_out;
        M3508_SetCurrent(&Motor_Output); //设置电机输出值

        EBFPIDDebuger_Set_Computer_Value(&pid_config_opts, SEND_FACT_CMD, CURVES_CH1, &M3508[0].Total_Angle, 4);
        EBFPIDDebuger_Set_Computer_Value(&pid_config_opts, SEND_FACT_CMD, CURVES_CH2, &M3508[1].Total_Angle, 4);
        EBFPIDDebuger_Set_Computer_Value(&pid_config_opts, SEND_FACT_CMD, CURVES_CH3, &M3508[2].Total_Angle, 4);
        EBFPIDDebuger_Set_Computer_Value(&pid_config_opts, SEND_FACT_CMD, CURVES_CH4, &M3508[3].Total_Angle, 4);

        EBFPIDDebuger_Receiving_Process(&pid_config_opts);
        osDelay(Delay_Time);
    }
}

#endif
