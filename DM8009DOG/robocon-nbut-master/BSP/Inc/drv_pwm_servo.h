#ifndef __DRV_PWM_SERVO_H
#define __DRV_PWM_SERVO_H

#ifdef __cplusplus
extern "C"
{
#endif /*__cplusplus*/

#include <stdint.h>

typedef struct
{
    void (*PWM_Servo_SetCompare)(uint32_t val);
}PWM_Servo_Operation_t;

typedef struct
{
    PWM_Servo_Operation_t *opt;
    int32_t PWM_Max_Angle;
    int32_t PWM_Min_Angle;
    uint32_t PWM_Max_Duty;
    uint32_t PWM_Min_Duty;
    uint32_t PWM_Period;
}PWM_Servo_t;

void PWM_Servo_Write(PWM_Servo_t *pwm_servo, uint32_t val);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /*__DRV_PWM_SERVO_H*/
