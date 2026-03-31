#ifndef ENGINER_TASK_H
#define ENGINER_TASK_H

#include "main.h"
#include "pid.h"
#include "CAN_receive.h"
#include "user_lib.h"

#define POSITION_RING_CONTROL     1        //位置环控制标志(勿改)
#define SPEED_RING_CONTROL     		0        //速度环控制标志(勿改)

#define INIT_CURRENT       1800   //初始化电流

//电机码盘值最大以及中值   M2006  M3508  M6020均相同
#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8191


//M6020 减速比1:1   机械角度值范围:0~8191
//M3508 减速比19:1  机械角度值范围:0~8191
//M2006 减速比36:1  机械角度值范围:0~8191
//电机编码值所对应的角度比    例:3508电机减速比为19:1,码盘最大值为8191，计算(2*π)/(19*8191)  单位rad
#define M6020_MOTOR_ECD_TO_ANGLE        0.00076708403f
#define M3508_MOTOR_ECD_TO_ANGLE        0.00004037284f
#define M2006_MOTOR_ECD_TO_ANGLE        0.00002130789f

/**************************************M6020电机************************************************/
//参数仅供参考，实际应用到机器人上应实际修改(勿修改此处参考值)
//M6020电机速度环PID
#define M6020_MOTOR_SPEED_PID_KP        40.0f
#define M6020_MOTOR_SPEED_PID_KI        2.0f
#define M6020_MOTOR_SPEED_PID_KD        0.2f
#define M6020_MOTOR_SPEED_PID_MAX_OUT   30000.0f      //最大输出电流
#define M6020_MOTOR_SPEED_PID_MAX_IOUT  5000.0f

//M6020电机位置环PID
#define M6020_MOTOR_POSITION_PID_KP        1500.0f  
#define M6020_MOTOR_POSITION_PID_KI        2.0f
#define M6020_MOTOR_POSITION_PID_KD        0.3f
#define M6020_MOTOR_POSITION_PID_MAX_OUT   1500.0f
#define M6020_MOTOR_POSITION_PID_MAX_IOUT  400.0f
/**************************************M6020电机************************************************/

/**************************************M3508电机************************************************/
//参数仅供参考，实际应用到机器人上应实际修改(勿修改此处参考值)
//M3508电机速度环PID
#define M3508_MOTOR_SPEED_PID_KP 20.0f
#define M3508_MOTOR_SPEED_PID_KI 1.0f
#define M3508_MOTOR_SPEED_PID_KD 0.0f
#define M3508_MOTOR_SPEED_PID_MAX_OUT 16384.0f        //最大输出电流
#define M3508_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//M3508电机位置环PID
#define M3508_MOTOR_POSITION_PID_KP 1000.0f
#define M3508_MOTOR_POSITION_PID_KI 2.0f
#define M3508_MOTOR_POSITION_PID_KD 0.0f
#define M3508_MOTOR_POSITION_PID_MAX_OUT 1500.0f
#define M3508_MOTOR_POSITION_PID_MAX_IOUT 400.0f
/**************************************M3508电机************************************************/

/**************************************M2006电机************************************************/
//参数仅供参考，实际应用到机器人上应实际修改(勿修改此处参考值)
//M2006电机速度环PID
#define M2006_MOTOR_SPEED_PID_KP 5.0f
#define M2006_MOTOR_SPEED_PID_KI 0.1f
#define M2006_MOTOR_SPEED_PID_KD 1.0f
#define M2006_MOTOR_SPEED_PID_MAX_OUT 10000.0f        //最大输出电流
#define M2006_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//M2006电机位置环PID
#define M2006_MOTOR_POSITION_PID_KP 1500.0f
#define M2006_MOTOR_POSITION_PID_KI 0.2f
#define M2006_MOTOR_POSITION_PID_KD 0.0f
#define M2006_MOTOR_POSITION_PID_MAX_OUT 3000.0f
#define M2006_MOTOR_POSITION_PID_MAX_IOUT 400.0f
/**************************************M2006电机************************************************/

typedef struct
{
    motor_measure_t *enginer_motor_measure;
		int16_t  target_speed;								//目标速度		rpm/s
		float target_angle;										//目标角度		rad
		float angle;													//累积角度   	rad
		int16_t give_current;        					//给的电流		
} enginer_motor_t;


typedef struct
{
    enginer_motor_t motor;    						//电机结构体
	  pid_type_def motor_speed_pid;        	//底盘电机速度pid
		pid_type_def motor_angle_pid;         //底盘跟随角度pid
} enginer_control_t;


typedef struct{
	
	fp32 vx_set;
	fp32 vy_set;
	fp32 wz_set;
	first_order_filter_type_t vx_fir;
	first_order_filter_type_t vy_fir;
	first_order_filter_type_t wz_fir;
} chassis_speed_t;


extern enginer_control_t enginer_move_can1[8];     		//CAN1需要控制的电机结构体
extern enginer_control_t enginer_move_can2[8];					//CAN2需要控制的电机结构体


//电机控制PID初始化函数 
void enginer_init(enginer_control_t *enginer_move_init, uint8_t MOTOR_ID, uint8_t CAN, fp32 SPEED_PID[3], fp32 SPEED_MAX_OUT, \
	fp32 SPEED_PID_MAX_IOUT, fp32 POSITION_PID[3], fp32 POSITION_PID_MAX_OUT, fp32 POSITION_PID_MAX_IOUT);

//电机PID控制实现函数
void PIDrealize(enginer_control_t *enginer_move, uint8_t MOTOR_MODEL, fp32 MOTOR_ECD_TO_ANGLE);

//映射函数
fp32 map(fp32 x, fp32 in_min, fp32 in_max, fp32 out_min, fp32 out_max);

void Chassis_Spd(const chassis_speed_t speed);

#endif
