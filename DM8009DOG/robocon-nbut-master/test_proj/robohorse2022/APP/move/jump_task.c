#include "jump_task.h"
//#include "stm32f4xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "gait_param.h"
#include "posture_task.h"

extern osThreadId_t JumpHandle;
extern PID_t PID_Speed[8]; //速度PID
extern PID_t PID_Position[8];

float start_time_jump = 0.0f;

void Jump_OnlyOnce(void);
void Polar_setCoord(Polar_Coord_Data_t *Action_Polar_Buffer);

void Jump_Task(void *argument)
{
    while (1)
    {

        static uint8_t flag_enterTwice = 0;
        /* 防止上电时运行任务 */
        if (flag_enterTwice == 0)
        { /* 有两个jump任务调用Jump_Task */
            flag_enterTwice++;
        }
        else
        {
            start_time_jump = HAL_GetTick(); //跳跃开始的时间
            Jump_OnlyOnce();
        }
        osThreadSuspend(JumpHandle);
    }
}

/**
 * @brief 跳跃一次，跳跃完成后会将NowState切换为STOP
 * @return void
 */
void Jump_OnlyOnce()
{
    const float prep_time = 0.8f;   // 准备时间 [s]		0.8
    const float launch_time = 0.2f; // 收缩腿前的持续时间 [s]		0.2
    const float fall_time = 0.8f;   //降落时的减速时间 [s]		0.8
    const float stabilize_time = 0.6f;

    float t = 0;
    /*************************************平地*************************************/
    if (NowState == JUMP_GROUND)
    {
        while (t <= prep_time + launch_time + fall_time + stabilize_time)
        {
            /* 极坐标 */
            Polar_Coord_Data_t Action_Polar[4] = {
                /*  r   θ */
                13, -13.0f,
                13, -13.0f,
                13, -13.0f,
                13, -13.0f};

            t = HAL_GetTick() / 1000.0f - start_time_jump / 1000.0f; // 跳跃开始后的时间
            /* 跳跃储能阶段 */
            if (t < prep_time)
            {
                PID_Reset(PID_Position, 5.0f, 0.0008f, 0.0f);
                PID_Reset(PID_Speed, 8.0f, 0.0001f, 0.0f);

                Polar_setCoord(Action_Polar);
            }
            /* 跳跃上升阶段 */
            else if (t >= prep_time && t < prep_time + launch_time)
            {
                for (uint8_t i = 0; i < 4; i++)
                {
                    Action_Polar[i].radius = 28.0f;
                    Action_Polar[i].theta = -20.0f;
                }

                PID_Reset(PID_Position, 22.0f, 0.0008f, 0.0f);
                PID_Reset(PID_Speed, 16.0f, 0.0001f, 0.0f);

                Polar_setCoord(Action_Polar);
            }
            /* 跳跃减速阶段 */
            else if (t >= prep_time + launch_time && t < prep_time + launch_time + fall_time)
            {
                for (uint8_t i = 0; i < 4; i++)
                {
                    Action_Polar[i].radius = 17.0f;
                    Action_Polar[i].theta = 7.0f;
                }

                PID_Reset(PID_Position, 15.0f, 0.0008f, 0.0f);
                PID_Reset(PID_Speed, 1.8f, 0.0001f, 0.0f);

                Polar_setCoord(Action_Polar);
            }
            /* 跳跃稳定阶段 */
            else if (t > prep_time + launch_time + fall_time && t < prep_time + launch_time + fall_time + stabilize_time)
            {
                for (uint8_t i = 0; i < 4; i++)
                {
                    Action_Polar[i].radius = 18.0f;
                    Action_Polar[i].theta = 4.0f;
                }

                PID_Reset(PID_Position, 15.0f, 0.0008f, 0.0f);
                PID_Reset(PID_Speed, 1.8f, 0.0001f, 0.0f);

                Polar_setCoord(Action_Polar);
            }
            /* 跳跃结束 */
            else if (t > prep_time + launch_time + fall_time + stabilize_time)
            {
                Change_NowState(STOP);
                PID_Reset(PID_Position, 8.0f, 0.0008f, 0.0f);
                PID_Reset(PID_Speed, 15.5f, 0.0001f, 0.0f);
            }
        }
    }
    /*************************************上坡*************************************/
    else if (NowState == JUMP_UPHILL)
    {
        while (t <= prep_time + launch_time + fall_time + stabilize_time)
        {
            /* 极坐标 */
            Polar_Coord_Data_t Action_Polar[4] = {
                /*  r   θ */
                13, -13.0f,
                13, -13.0f,
                13, -13.0f,
                13, -13.0f};

            t = HAL_GetTick() / 1000.0f - start_time_jump / 1000.0f; // 跳跃开始后的时间
            /* 跳跃储能阶段 */
            if (t < prep_time)
            {
                PID_Reset(PID_Position, 5.0f, 0.0008f, 0.0f);
                PID_Reset(PID_Speed, 8.0f, 0.0001f, 0.0f);

                Polar_setCoord(Action_Polar);
            }
            /* 跳跃上升阶段 */
            else if (t >= prep_time && t < prep_time + launch_time)
            {
                for (uint8_t i = 0; i < 4; i++)
                {
                    Action_Polar[i].radius = 27.6f;
                    Action_Polar[i].theta = -18.0f;
                }

                PID_Reset(PID_Position, 28.0f, 0.0008f, 0.0f);
                PID_Reset(PID_Speed, 16.0f, 0.0001f, 0.0f);

                Polar_setCoord(Action_Polar);
            }
            /* 跳跃减速阶段 */
            else if (t >= prep_time + launch_time && t < prep_time + launch_time + fall_time)
            {
                for (uint8_t i = 0; i < 4; i++)
                {
                    Action_Polar[i].radius = 16.0f;
                    Action_Polar[i].theta = 0.0f;
                }
                
                PID_Reset(PID_Position, 5.0f, 0.0008f, 0.0f);
                PID_Speed[0].MaxOutput = 6500.0f;
                PID_Speed[1].MaxOutput = 6500.0f;
                PID_Speed[2].MaxOutput = 6500.0f;
                PID_Speed[3].MaxOutput = 6500.0f;
                PID_Speed[4].MaxOutput = 6500.0f;
                PID_Speed[5].MaxOutput = 6500.0f;
                PID_Speed[6].MaxOutput = 6500.0f;
                PID_Speed[7].MaxOutput = 6500.0f;
                
                PID_Reset(PID_Speed, 1.8f, 0.0001f, 0.0f);

                Polar_setCoord(Action_Polar);
            }
            /* 跳跃稳定阶段 */
            else if (t > prep_time + launch_time + fall_time && t < prep_time + launch_time + fall_time + stabilize_time)
            {
                for (uint8_t i = 0; i < 4; i++)
                {
                    Action_Polar[i].radius = 16.0f;
                    Action_Polar[i].theta = 0.0f;
                }

                PID_Reset(PID_Position, 8.0f, 0.0008f, 0.0f);
                PID_Reset(PID_Speed, 1.8f, 0.0001f, 0.0f);

                Polar_setCoord(Action_Polar);
            }
            /* 跳跃结束 */
            else if (t > prep_time + launch_time + fall_time + stabilize_time)
            {
                PID_Speed[0].MaxOutput = 10000.0f;
                PID_Speed[1].MaxOutput = 10000.0f;
                PID_Speed[2].MaxOutput = 10000.0f;
                PID_Speed[3].MaxOutput = 10000.0f;
                PID_Speed[4].MaxOutput = 10000.0f;
                PID_Speed[5].MaxOutput = 10000.0f;
                PID_Speed[6].MaxOutput = 10000.0f;
                PID_Speed[7].MaxOutput = 10000.0f;

                Change_NowState(STOP);
                PID_Reset(PID_Position, 8.0f, 0.0008f, 0.0f);
                PID_Reset(PID_Speed, 15.5f, 0.0001f, 0.0f);
            }
        }
    }
}
/**
 * @brief 将目标极坐标转化成电机角度并输出到电机
 * @param Action_Polar_Buffer 目标极坐标 数组长度必须为4
 * @return void
 */
void Polar_setCoord(Polar_Coord_Data_t *Action_Polar_Buffer)
{
    /* 直角坐标暂存区 */
    static Cart_Coord_Data_t Action_Cart_Buffer[4] = {0};

    for (uint8_t i = 0; i < 4; i++)
    {
        Polar_toCartesian(&Action_Polar_Buffer[i], &Action_Cart_Buffer[i]);
        Stanford_Type_Lite_SetPosition(&Gait_Data[i], Action_Cart_Buffer[i].cx, Action_Cart_Buffer[i].cy); //运动学解算待数据填充
        Stanford_Type_Lite_Inverse_Kinematics(&Gait_Data[i]);
        //		Stanford_Type_Lite_Forward_Kinematics(&Gait_Data[i]);
        Aim_Angle[i * 2] = -(pi / 2.0f - Gait_Data[i].angle[1]) / 1.0f; //-顺时针,弧度
        Aim_Angle[i * 2 + 1] = -(pi / 2.0f + Gait_Data[i].angle[0]) / 1.0f;
    }
    Is_Aim_Angle_Get = 1; //数据填充完毕
}
