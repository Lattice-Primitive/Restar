#ifndef DM8009_H
#define DM8009_H

#include <stdint.h>

///**
// * @brief M3508输出电流结构体
// */
//typedef struct
//{
//    int16_t ID1;
//    int16_t ID2;
//    int16_t ID3;
//    int16_t ID4;
//    int16_t ID5;
//    int16_t ID6;
//    int16_t ID7;
//    int16_t ID8;
//} M3508_OutputCurrent_t;

//typedef struct
//{
//    int16_t Rotor_Angle;  //转子角度 0~8192 0~360°
//    int16_t Speed_RPM;    //每分钟电机转速(RPM)
//    int16_t Real_Current; //实际电机电流值 -16384~+16384
//    uint16_t Temp;        //电机温度
//    int16_t Last_Angle;   //上次读取到的角度
//    int16_t Offset_Angle; //初始位置偏移角度
//    int32_t Round_Cnt;    //转子旋转圈数
//    int32_t Total_Angle;  //总角度
//    uint32_t Msg_Cnt;     //消息计数
//} M3508_Measure_t;        //电机测量数据结构体

void DM8009_Init(void);

///*电机输出电流控制*/
///**
// * @brief 电机输出电流控制
// * @param currents 各电机电流值结构体
// */
//void M3508_SetCurrent(M3508_OutputCurrent_t *currents);

#endif
