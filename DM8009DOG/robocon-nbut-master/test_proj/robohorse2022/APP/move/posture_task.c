#include "posture_task.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "hwt906.h"
#include "serial.h"
#include "ahrs_task.h"

float aim_x[4] = {0.0f, 0.0f, 0.0f, 0.0f}, aim_y[4] = {0.0f, 0.0f, 0.0f, 0.0f}; //目标点暂存
float now_time = 0.0f;                                                          //目前时间 (用于PLAN A)

int time_cnt = 0; //时间计数器（用于PLAN B）
extern float revise_LegLenth[4];
extern osThreadId_t JumpHandle;
extern osThreadId_t JumpUphillHandle;
extern uint8_t flag_Standup;
extern uint8_t flag_imu_line;

extern void Ctrl_byRemoctrl(Cycloid_Generator_t *pCycGenerato, RC_ctrl_t *pRCData);

void Calculate_Gait(Robohorse_State state);
void Target_toMotor(void);

/**
 * 步态控制任务
 * 流程：获取当前模式->[轨迹生成器]获得坐标->运动学逆解获得目标弧度
 * */
void Posture_Task(void *argument)
{
    while (1)
    {
        gait_params = state_gait_params[NowState];         //提取步态参数表（通用）的参数
        RcDetachedParam = state_detached_params[NowState]; //提取步态参数表（分离）的参数

        if (Mode_Change_Flag)
        {
            Last_Time = HAL_GetTick() / 1000.0f; //获取当前运行时间作为开始时间
            for (uint8_t i = 0; i < 4; i++)
            {
                Cycloid_Gen[i].last_time = Last_Time;
            }
        }
        for (uint8_t i = 0; i < 4; i++)
        {
            Cycloid_Gen[i].param = gait_params; //步态参数
        }

        Ctrl_byRemoctrl(Cycloid_Gen, &my_rc_ctrl);

        switch (NowState) //获取当前模式
        {
        // 停止并直立状态
        case STOP://停止
            for (uint8_t i = 0; i < 4; i++)
            {
                aim_x[i] = 0.0f; // 此时马腿呈直立态
                aim_y[i] = 18.0f;
            }
            Target_toMotor();
            break;
        case CLIMBING://爬坡
            Calculate_Gait(CLIMBING);
            Target_toMotor();
            break;
        case WALK://慢步
            Calculate_Gait(WALK);
            Target_toMotor();
            break;
        case TROT://小跑
            Calculate_Gait(TROT);
            Target_toMotor();
            break;
        case BRIDGE://桥
            Calculate_Gait(TROT);
            Target_toMotor();
        case JUMP_GROUND://跳跃
            osThreadResume(JumpHandle);
            break;
        case JUMP_UPHILL://跳跃上坡
            osThreadResume(JumpHandle);
            break;
        // 保持上一个动作0.5s
        case RELEASE:
        default:
            osDelay(500);
            break;
        }

        /*绝对延时*/
        static portTickType xLastWakeTime;
        static const portTickType xFrequency = pdMS_TO_TICKS(10);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/**
 * @brief 切换步态
 * @param target 目标步态
 * @return void
 */
void Change_NowState(Robohorse_State target)
{
    if (flag_Standup)
    {
        if (NowState != target)
        {
            taskENTER_CRITICAL();
            NowState = target;
            Mode_Change_Flag = 1;
            taskEXIT_CRITICAL();
        }
    }
    else
        return;
}
/**
 * @brief 步态参数计算
 * @param state
 * @return void
 */
void Calculate_Gait(Robohorse_State state)
{
    now_time = HAL_GetTick() / 1000.0f; //获取当前时间用于后续运算
    Cycloid_Generator_Solution_t cyc_solution[4];
    switch (state)
    {
        // FIXME:Cycloid_Generator()第三个参数不能为0 否则切换步态会导致一条腿残废
    case WALK:
        if (flag_imu_line == 1)
        {
            Cycloid_Gen[0].param.step_length -= step_len_dev;
            Cycloid_Gen[1].param.step_length -= step_len_dev;
            Cycloid_Gen[2].param.step_length += step_len_dev;
            Cycloid_Gen[3].param.step_length += step_len_dev;
            LIMIT_PARAM(Cycloid_Gen[0].param.step_length, 16);
            LIMIT_PARAM(Cycloid_Gen[1].param.step_length, 16);
            LIMIT_PARAM(Cycloid_Gen[2].param.step_length, 16);
            LIMIT_PARAM(Cycloid_Gen[3].param.step_length, 16);
        }
        //轨迹生成器运算
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[0], 0.01f, &cyc_solution[0]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[1], 0.21f, &cyc_solution[1]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[2], 0.51f, &cyc_solution[2]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[3], 0.71f, &cyc_solution[3]);
        break;

    case CLIMBING:
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[0], 0.01f, &cyc_solution[0]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[1], 0.51f, &cyc_solution[1]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[2], 0.51f, &cyc_solution[2]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[3], 0.01f, &cyc_solution[3]);
        break;
    
    case TROT:
    case BRIDGE:
        if (flag_imu_line == 1)
        {
            Cycloid_Gen[0].param.step_length -= step_len_dev;
            Cycloid_Gen[1].param.step_length -= step_len_dev;
            Cycloid_Gen[2].param.step_length += step_len_dev;
            Cycloid_Gen[3].param.step_length += step_len_dev;
            LIMIT_PARAM(Cycloid_Gen[0].param.step_length, 16);
            LIMIT_PARAM(Cycloid_Gen[1].param.step_length, 16);
            LIMIT_PARAM(Cycloid_Gen[2].param.step_length, 16);
            LIMIT_PARAM(Cycloid_Gen[3].param.step_length, 16);
        }
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[0], 0.01f, &cyc_solution[0]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[1], 0.51f, &cyc_solution[1]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[2], 0.51f, &cyc_solution[2]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[3], 0.01f, &cyc_solution[3]);
        break;

    default:
        break;
    }

    //装填结果值 弧度rad
    for (uint8_t i = 0; i < 4; i++)
    {
        aim_x[i] = cyc_solution[i].x;
        aim_y[i] = cyc_solution[i].y;
    }
}
/**
 * @brief 目标坐标转化角度并输出到电机
 *
 */
void Target_toMotor()
{
    for (uint8_t i = 0; i < 4; i++)
    {
        Stanford_Type_Lite_SetPosition(&Gait_Data[i], aim_x[i], aim_y[i]); //运动学解算待数据填充
        Stanford_Type_Lite_Inverse_Kinematics(&Gait_Data[i]);              //运动学逆解算 坐标转弧度

        // 修改电机ID映射关系
        // 右前腿(0): 电机0,1
        // 右后腿(1): 电机2,3
        // 左前腿(2): 电机4,5
        // 左后腿(3): 电机6,7
        if (i == 0) { // 右前腿
            Aim_Angle[0] = -(pi / 2.0f - Gait_Data[i].angle[1]) / 1.0f;     //外侧电机
            Aim_Angle[1] = -(pi / 2.0f + Gait_Data[i].angle[0]) / 1.0f;     //内侧电机
        }
        else if (i == 1) { // 右后腿
            Aim_Angle[2] = -(pi / 2.0f - Gait_Data[i].angle[1]) / 1.0f;     //外侧电机
            Aim_Angle[3] = -(pi / 2.0f + Gait_Data[i].angle[0]) / 1.0f;     //内侧电机
        }
        else if (i == 2) { // 左前腿
            Aim_Angle[4] = -(pi / 2.0f - Gait_Data[i].angle[1]) / 1.0f;     //外侧电机
            Aim_Angle[5] = -(pi / 2.0f + Gait_Data[i].angle[0]) / 1.0f;     //内侧电机
        }
        else if (i == 3) { // 左后腿
            Aim_Angle[6] = -(pi / 2.0f - Gait_Data[i].angle[1]) / 1.0f;     //外侧电机
            Aim_Angle[7] = -(pi / 2.0f + Gait_Data[i].angle[0]) / 1.0f;     //内侧电机
        }
    }
    Is_Aim_Angle_Get = 1; //数据填充完毕
}
