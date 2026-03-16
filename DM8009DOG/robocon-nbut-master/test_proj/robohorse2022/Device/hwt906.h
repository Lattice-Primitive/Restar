#ifndef HWT906_H
#define HWT906_H

#include "pid.h"

/*陀螺仪欧拉较结构体*/
typedef struct 
{
    float Roll; //横滚角
    float Pitch; //俯仰角
    float Yaw; //偏航角
}AHRS_Angle_t;

extern PID_t PID_IMU[3]; //陀螺仪PID
extern AHRS_Angle_t Now_Angle; //当前角度

void HWT906_Init(void);

void HWT906_Read(AHRS_Angle_t *Now_Angle);

#endif
