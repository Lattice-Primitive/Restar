/*步态参数设置*/
#include "gait_param.h"

LegGain Gait_Gains = {15.5f, 0.00f, 25.0f, 0.00f};

LegGain state_gait_gains[END+1] = {
    //{kp_pos, kd_pos} kp_spd, kd_spd,
    {22.0f, 0.00f, 4.0f, 0.00f}, //位置环 速度环 TROT

    {22.0f, 0.00f, 4.0f, 0.00f},
    {22.0f, 0.00f, 4.0f, 0.00f},
    {22.0f, 0.00f, 4.0f, 0.00f},
    {22.0f, 0.00f, 4.0f, 0.00f},

    {22.0f, 0.00f, 4.0f, 0.00f},
    {22.0f, 0.00f, 4.0f, 0.00f},
    {22.0f, 0.00f, 4.0f, 0.00f},
    {22.0f, 0.00f, 4.0f, 0.00f},
    {22.0f, 0.00f, 4.0f, 0.00f},
};

Cycloid_Generator_Param_t gait_params = {20.0f, 0.0f, 3.0f, 0.00f, 0.25f, 0.3f};
// typedef struct
//    {
//        float body_height;    //身体到地面的距离 (cm)
//        float step_length;    //一步的距离 (cm)
//        float up_amp;         //上部振幅y (cm)
//        float down_amp;       //下部振幅 (cm)
//        float flight_percent; //摆动相百分比(%)
//        float freq;           //一步的频率 (Hz)
//    } Cycloid_Generator_Param_t;
Cycloid_Generator_Param_t state_gait_params[] = {
	  // NORMAL
    {16.0f, 0.0f, 0.00f, 0.00f, 0.0f, 0.1f},    
                                               
		// CLIMBING 爬坡
    {20.3f, 2.30f, 6.50f, 0.1f, 0.3f, 3.5f}, 
		//{18.0f, 10.00f, 3.3f, 0.0f, 0.25f, 1.0f},
		
		 // TURN_LEFT 
    {20.0f, 7.00f, 2.5f, 2.00f, 0.25f, 3.0f},
		//{22.0f, 7.00f, 4.0f, 2.00f, 0.28f, 1.0f}, 
		
		// TURN_RIGHT
    {20.0f, 7.00f, 2.5f, 2.00f, 0.25f, 3.0f},       
		
		// WALK
//		{20.3f, 7.30f, 1.30f, 2.6f, 0.45f, 3.5f}, 
		{20.3f, 7.30f, 6.50f, 0.1f, 0.45f, 3.5f},

   // {20.0f, 9.00f, 2.0f, 0.1f, 0.45f, 2.0f},   
    //目前最好值  12
		
		// 低趴 （trot）
    {13.99f, 6.00f, 2.5f, 0.1f, 0.5f, 3.5f},   
		//{18.0f, 10.00f, 4.3f, 1.0f, 0.25f, 3.6f},TROT快
		
		
		// BRIDG闪光黑子
    {19.0f, 10.00f, 3.3f, 0.1f, 0.4f, 0.7f}, 
                                            
		// STOP    
    {20.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},   
		
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
        {22.0f, 7.00f, 4.0f, 2.00f, 0.28f, 3.0f}, // TURN_LEFT
        {22.0f, 7.00f, 4.0f, 2.00f, 0.28f, 3.0f}, // TURN_LEFT
        {22.0f, 7.00f, 4.0f, 2.00f, 0.28f, 3.0f}, // TURN_LEFT
        {22.0f, 7.00f, 4.0f, 2.00f, 0.28f, 3.0f}  // TURN_LEFT
    },
    {
        {22.0f, 7.00f, 4.0f, 2.00f, 0.28f, 3.0f}, // TURN_RIGHT
        {22.0f, 7.00f, 4.0f, 2.00f, 0.28f, 3.0f}, // TURN_RIGHT
        {22.0f, 7.00f, 4.0f, 2.00f, 0.28f, 3.0f}, // TURN_RIGHT
        {22.0f, 7.00f, 4.0f, 2.00f, 0.28f, 3.0f}  // TURN_RIGHT
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

// 1. 计算body_height
//float min_height = 12.0f;
//float max_height = 22.0f;
//float norm = (pRCData->rc.ch[4] - 400) / 1200.0f; // 0~1
//if (norm < 0) norm = 0;
//if (norm > 1) norm = 1;
//float body_height = min_height + norm * (max_height - min_height);

//// 2. 赋值给所有腿
//for (int i = 0; i < 4; i++) {
//    pCycGenerato[i].param.body_height = body_height;
//}
