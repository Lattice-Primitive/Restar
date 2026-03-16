/*步态参数设置*/
#include "gait_param.h"

LegGain Gait_Gains = {15.5f, 0.00f, 25.0f, 0.00f};

LegGain state_gait_gains[] = {
    //{kp_pos, kd_pos} kp_spd, kd_spd,
    {22.0f, 0.00f, 8.0f, 0.00f}, //位置环 速度环 TROT

    {22.0f, 0.00f, 8.0f, 0.00f},
    {22.0f, 0.00f, 8.0f, 0.00f},
    {22.0f, 0.00f, 8.0f, 0.00f},
    {22.0f, 0.00f, 8.0f, 0.00f},

    {22.0f, 0.00f, 8.0f, 0.00f},
    {22.0f, 0.00f, 8.0f, 0.00f},
    {22.0f, 0.00f, 8.0f, 0.00f},
    {22.0f, 0.00f, 8.0f, 0.00f},
    {22.0f, 0.00f, 8.0f, 0.00f},
};

Cycloid_Generator_Param_t gait_params = {17.3f, 0.0f, 3.0f, 0.00f, 0.25f, 0.3f};

Cycloid_Generator_Param_t state_gait_params[] = {
    {16.0f, 0.0f, 0.00f, 0.00f, 0.0f, 0.1f},    // NORMAL
                                                //    {18.0f, 10.00f, 3.3f, 0.0f, 0.25f, 1.0f},
    {18.3f, 12.00f, 5.20f, 3.20f, 0.30f, 3.5f}, // CLIMBING 爬坡
    {23.0f, 7.00f, 3.0f, 2.00f, 0.25f, 3.0f},   // TURN_LEFT
    {21.0f, 7.00f, 5.0f, 2.00f, 0.25f, 3.0f},   // TURN_RIGHT
    {18.0f, 12.00f, 3.4f, 0.1f, 0.25f, 1.0f},   // WALK 目前最好值  12

    {18.0f, 10.00f, 4.5f, 1.2f, 0.25f, 2.5f}, // TROT     10
    {18.0f, 10.00f, 4.3f, 1.0f, 0.25f, 0.7f}, // BRIDGE
                                              //		{18.0f, 10.00f, 4.3f, 1.0f, 0.25f, 3.6f},   // TROT快
    {16.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},    // STOP
    {NAN, NAN, NAN, NAN, NAN, NAN},           // RELASE
    {NAN, NAN, NAN, NAN, NAN, NAN},           // JUMP_GROUND
    {NAN, NAN, NAN, NAN, NAN, NAN},           // JUMP_UPHILL
    {NAN, NAN, NAN, NAN, NAN, NAN},           // START
    {NAN, NAN, NAN, NAN, NAN, NAN},           // END
};

DetachedParam detached_params;
DetachedParam state_detached_params[] = {

    {                                           // TROT 对角小跑
     {21.0f, 14.0f, 3.00f, 2.00f, 0.25f, 4.0f}, //   {21.0, 12.0, 3.00, 2.00, 0.25, 4.0}
     {21.0f, 14.0f, 3.00f, 2.00f, 0.25f, 4.0f},
     {21.0f, 14.0f, 3.00f, 2.00f, 0.25f, 4.0f},
     {21.0f, 14.0f, 3.00f, 2.00f, 0.25f, 4.0f}},

    {                                          // CLIMBING	爬坡
     {16.0f, 9.0f, 3.00f, 1.00f, 0.25f, 3.6f}, //小步子可以迈上去
     {16.0f, 9.0f, 3.00f, 1.00f, 0.25f, 3.6f},
     {18.0f, 9.0f, 3.00f, 1.00f, 0.25f, 3.6f},
     {18.0f, 9.0f, 3.00f, 1.00f, 0.25f, 3.6f}},
    {
        {23.0f, 7.00f, 3.0f, 2.00f, 0.25f, 3.0f}, // TURN_LEFT
        {23.0f, 7.00f, 3.0f, 2.00f, 0.25f, 3.0f}, // TURN_LEFT
        {23.0f, 7.00f, 3.0f, 2.00f, 0.25f, 3.0f}, // TURN_LEFT
        {23.0f, 7.00f, 3.0f, 2.00f, 0.25f, 3.0f}  // TURN_LEFT
    },
    {
        {21.0f, 7.00f, 5.0f, 2.00f, 0.25f, 3.0f}, // TURN_RIGHT
        {21.0f, 7.00f, 5.0f, 2.00f, 0.25f, 3.0f}, // TURN_RIGHT
        {21.0f, 7.00f, 5.0f, 2.00f, 0.25f, 3.0f}, // TURN_RIGHT
        {21.0f, 7.00f, 5.0f, 2.00f, 0.25f, 3.0f}  // TURN_RIGHT
    },

};

DetachedParam RcDetachedParam = {
    {21.0f, 0.0f, 3.00f, 2.00f, 0.25f, 3.2f},
    {21.0f, 0.0f, 3.00f, 2.00f, 0.25f, 3.2f},
    {21.0f, 0.0f, 3.00f, 2.00f, 0.25f, 3.2f},
    {21.0f, 0.0f, 3.00f, 2.00f, 0.25f, 3.2f}};

Robohorse_State NowState = STOP; //当前运行状态
Robohorse_Direction NowDir = POSITIVE;

Stanford_Type_Lite_Data_t Gait_Data[4]; // 步态数据

Cycloid_Generator_t Cycloid_Gen[4]; // 摆线生成器

float Aim_Angle[8]; //目标角度，单位为弧度rad

volatile uint8_t Is_Aim_Angle_Get = 0; //是否已经获得到目标角度

float Last_Time = 0.0f; //上次计算时间

uint8_t Mode_Change_Flag = 1; //模式切换标志
