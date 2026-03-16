//3508
#include <stdarg.h>
#include <stdint.h>

#include "motor_task.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "dm8009.h"
#include "pid.h"
#include "gait_param.h"
#include "usart.h"
#include "app_ebf_piddebug.h"

#include "standup_task.h"
#include "dm_j8009_ctrl.h"

extern PID_t PID_Position[8];    //位置环PID
extern PID_t PID_Speed[8];       //速度环PID

extern float imu_Leg_offset;     //陀螺仪偏移量
float PID_Aim_Angle[8]; //将用于PID运算的目标角度

uint8_t DM_J8009_ENABLE_FLAG = 1;
uint8_t DM_J8009_ctrl_FLAG = 0;
#define PID_CONFIG 0

#if PID_CONFIG == 0

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
//                if (i == 0 || i == 1) { // 右前腿
//                    PID_Aim_Angle[i] = Aim_Angle[i] + Standup_LegR_Offset - imu_Leg_offset;
//                }
//                else if (i == 2 || i == 3) { // 右后腿
//                    PID_Aim_Angle[i] = Aim_Angle[i] + Standup_LegR_Offset - imu_Leg_offset;
//                }
//                else if (i == 4 || i == 5) { // 左前腿
//                    PID_Aim_Angle[i] = Aim_Angle[i] + Standup_LegL_Offset + imu_Leg_offset;
//                }
//                else if (i == 6 || i == 7) { // 左后腿
//                    PID_Aim_Angle[i] = Aim_Angle[i] + Standup_LegL_Offset + imu_Leg_offset;
//                }

                // if (i % 2 == 0)
                //     PID_Aim_Angle[i] = Aim_Angle[i] + Standup_LegR_Offset - imu_Leg_offset; //数据转移
                // else
                //     PID_Aim_Angle[i] = Aim_Angle[i] + Standup_LegL_Offset + imu_Leg_offset; //数据转移
                if (i == 3)
                    PID_Aim_Angle[i] = Aim_Angle[i] - Standup_LegR_Offset - imu_Leg_offset; //数据转移
                if (i == 2)
                    PID_Aim_Angle[i] = Aim_Angle[i] + Standup_LegL_Offset + imu_Leg_offset; //数据
                if (i == 0)
                    PID_Aim_Angle[i] = Aim_Angle[i] + Standup_LegR_Offset - imu_Leg_offset; //数据转移
                if (i == 1)
                    PID_Aim_Angle[i] = Aim_Angle[i] - Standup_LegL_Offset + imu_Leg_offset; //数据
                if (i == 5)
                    PID_Aim_Angle[i] = Aim_Angle[i] + Standup_LegR_Offset - imu_Leg_offset; //数据转移
                if (i == 4)
                    PID_Aim_Angle[i] = Aim_Angle[i] - Standup_LegL_Offset + imu_Leg_offset; //数据
                if (i == 7)
                    PID_Aim_Angle[i] = Aim_Angle[i] - Standup_LegR_Offset - imu_Leg_offset; //数据转移
                if (i == 6)
                    PID_Aim_Angle[i] = Aim_Angle[i] + Standup_LegL_Offset + imu_Leg_offset; //数据
          
            }
            Is_Aim_Angle_Get = 0; //清零标志
        }

//        for (uint8_t i = 0; i < 8; i++) //双环PID运算
//        {
//            // 位置PID
//            PID_Calc(&PID_Position[i], M3508[i].Total_Angle * 360 / 8191, PID_Aim_Angle[i] * ReductionAndAngleRatio);
//            /* 限制位置环输出 */
//            //            LIMIT_PARAM(PID_Position[i].pos_out, 600);
//            // 速度PID
//            PID_Calc(&PID_Speed[i], M3508[i].Speed_RPM, PID_Position[i].pos_out);
//        }
//        Motor_Output.ID1 = PID_Speed[0].pos_out;
//        Motor_Output.ID2 = PID_Speed[1].pos_out;
//        Motor_Output.ID3 = PID_Speed[2].pos_out;
//        Motor_Output.ID4 = PID_Speed[3].pos_out;
//        Motor_Output.ID5 = PID_Speed[4].pos_out;
//        Motor_Output.ID6 = PID_Speed[5].pos_out;
//        Motor_Output.ID7 = PID_Speed[6].pos_out;
//        Motor_Output.ID8 = PID_Speed[7].pos_out;
//        M3508_SetCurrent(&Motor_Output); //设置电机输出值
				//dm电机最终控制实现
				 dm_j8009_all_ctrl(PID_Aim_Angle,PID_Position);
				
        /* 绝对延时 */
        static portTickType xLastWakeTime;
        static const portTickType xFrequency = pdMS_TO_TICKS(3);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        //        osDelay(3);
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
