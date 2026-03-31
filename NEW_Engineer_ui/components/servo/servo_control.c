#include "servo_control.h"
#include "bsp_servo_pwm.h"
#include "main.h"
#include "remote_control.h"
#include "enginer_task.h"
#include "keycontrol.h"

#define YAW_SERVO_MIN_PWM   500
#define YAW_SERVO_MAX_PWM   2500

#define PITCH_SERVO_MIN_PWM   1700
#define PITCH_SERVO_MAX_PWM   1900

#define PWM_DETAL_VALUE 100

#define YAW_SERVO_ID   1
#define PITCH_SERVO_ID 0

uint16_t yaw_pwm = 1500,pitch_pwm = 1700;

void servo_control_gimbal(void)
{
//	if(keyboard_ctrl.keyboard_E_flag)
//		yaw_pwm += 5;
//	else if(keyboard_ctrl.friction_flag)
//		yaw_pwm -= 5;
	pitch_pwm += map(rc_ctrl.mouse.y,-1000,1000,-PWM_DETAL_VALUE,PWM_DETAL_VALUE);
	
//	if(yaw_pwm < YAW_SERVO_MIN_PWM)
//		yaw_pwm = YAW_SERVO_MIN_PWM;
//	else if(yaw_pwm > YAW_SERVO_MAX_PWM)
//		yaw_pwm = YAW_SERVO_MAX_PWM;
	
	if(pitch_pwm < PITCH_SERVO_MIN_PWM)
		pitch_pwm = PITCH_SERVO_MIN_PWM;
	else if(pitch_pwm > PITCH_SERVO_MAX_PWM)
		pitch_pwm = PITCH_SERVO_MAX_PWM;
	
	servo_pwm_set(yaw_pwm,YAW_SERVO_ID);
	servo_pwm_set(pitch_pwm,PITCH_SERVO_ID);	
}



































