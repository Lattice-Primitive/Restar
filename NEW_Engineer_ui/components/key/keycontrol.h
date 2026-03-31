#ifndef KEYCONTROL_H
#define KEYCONTROL_H

#include "main.h"
#include "keyboard.h"
#include "enginer_task.h"
#include "user_lib.h"

#define CHASSIS_ACCEL_X_NUM 0.25f//0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.25f
#define CHASSIS_ACCEL_Z_NUM 0.3333333333f
//#define CHASSIS_CONTROL_TIME 0.002f
#define CHASSIS_CONTROL_TIME 0.05f

#define CHASSIS_CONTROL_TIME_WZ 0.1f

#define CHASSIS_SPEED 5000
//速度
#define FIRST_SPEED  1600  //1600
#define SECOND_SPEED 567   //567

#define X_AXLE_SPEED 3000
#define CONV_SPEED 2000

#define EXTEND_SPEED 2500

#define BIG_P_SPEED 500
#define BIG_R_SPEED 1500

#define SMALL_P_SPEED 2000//1500
#define SMALL_R_SPEED 1500

#define WZ_SPEED 5000
#define WZ_MOUSE 1000

#define WAVE_PEAK_LIMIT  100
#define WAVE_FIRST_FLAG  10
//抬升判断分界值
#define WAVE_RUN_FLAG    20

#define WAVE_XAXLE_LIMIT 150
#define WAVE_X_FST_FLAG  50
//x轴 判断分界值
#define WAVE_XAXLE_FLAG  100
//小p t=10ms 
#define SMALL_P_DELAY_SPEED 100

typedef enum{
	FLAG_START = 0,
	FLAG_FINISH
}flag_state_t;

typedef struct{
	_Bool first_curve_flag,first_curve_record,motor_run_flag;
}flag_switch_t;

typedef struct{
	uint16_t cal_rpm_temp,cal_speed_temp,error_abs;
	int16_t error_speed,last_error_speed;
	fp32 derror,derror_abs,start_time,last_time,hold_time;
}spd_limit_analog_switch_t;

extern chassis_speed_t chassis_spd;

void key_control_mode(void);

void first_init(void);






































#endif
