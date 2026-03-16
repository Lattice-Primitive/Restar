/**
 * @file pid_old.h 以前用的PID算法
 * @author Aoming Yu(2016th)
 * @brief 之前用的PID算法，新的用不惯可以使用老的
*/
#ifndef __PID_OLD_H
#define __PID_OLD_H

#ifdef __cplusplus
extern "C"
{
#endif /*__cplusplus*/

#include <stdint.h>

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    //PID 三参数
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //最大输出
    float max_iout; //最大积分输出

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //微分项 0最新 1上一次 2上上次
    float error[3]; //误差项 0最新 1上一次 2上上次

} PidTypeDef;

extern void PID_Init(PidTypeDef *pid, uint8_t mode, const float PID[3], float max_out, float max_iout);

extern float PID_Calc(PidTypeDef *pid, float ref, float set);

extern void PID_clear(PidTypeDef *pid);

float Limit_Max(float input, float max);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif
