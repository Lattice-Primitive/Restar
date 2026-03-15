#ifndef ENGINER_TASK_H
#define ENGINER_TASK_H

#include "main.h"
#include "pid.h"
#include "CAN_receive.h"


#define INIT_CURRENT       1800

//电机码盘值最大以及中值
#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8191

//M6020 减速比1:1   机械角度值范围:0~8191
//M3508 减速比19:1  机械角度值范围:0~8191
//M2006 减速比36:1  机械角度值范围:0~8191
//电机编码值所对应的角度比    例:3508电机减速比为19:1,码盘最大值为8191，计算(2*π)/(19*8191)  单位rad
#define MOTOR_ECD_TO_ANGLE_6020        0.00076708403f
#define MOTOR_ECD_TO_ANGLE_3508        0.00004037284f
#define MOTOR_ECD_TO_ANGLE_2006        0.00002130789f


//M6020电机速度环PID
#define M6020_MOTOR_SPEED_PID_KP        40.0f
#define M6020_MOTOR_SPEED_PID_KI        2.0f
#define M6020_MOTOR_SPEED_PID_KD        0.2f
#define M6020_MOTOR_SPEED_PID_MAX_OUT   30000.0f      //最大输出电流
#define M6020_MOTOR_SPEED_PID_MAX_IOUT  5000.0f

//M6020电机角度环PID
#define M6020_MOTOR_POSITION_PID_KP        1500.0f  
#define M6020_MOTOR_POSITION_PID_KI        2.0f
#define M6020_MOTOR_POSITION_PID_KD        0.3f
#define M6020_MOTOR_POSITION_PID_MAX_OUT   1500.0f
#define M6020_MOTOR_POSITION_PID_MAX_IOUT  400.0f


//M3508电机速度环PID
#define M3508_MOTOR_SPEED_PID_KP 10.0f
#define M3508_MOTOR_SPEED_PID_KI 1.0f
#define M3508_MOTOR_SPEED_PID_KD 0.0f
#define M3508_MOTOR_SPEED_PID_MAX_OUT 16000.0f        //最大输出电流
#define M3508_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//M3508电机角度环PID
#define M3508_MOTOR_POSITION_PID_KP 1500.0f
#define M3508_MOTOR_POSITION_PID_KI 20.0f
#define M3508_MOTOR_POSITION_PID_KD 0.0f
#define M3508_MOTOR_POSITION_PID_MAX_OUT 1500.0f
#define M3508_MOTOR_POSITION_PID_MAX_IOUT 400.0f


//M2006电机速度环PID
#define M2006_MOTOR_SPEED_PID_KP 5.0f
#define M2006_MOTOR_SPEED_PID_KI 0.1f
#define M2006_MOTOR_SPEED_PID_KD 1.0f
//p 2.0 \2.2 \2.3 2.5 3.0
//i 0.1 /0.11/ 0.12
//d 0.5 0.7 /1.0 /1.2
//#define M2006_MOTOR_SPEED_PID_KP 2.2f
//#define M2006_MOTOR_SPEED_PID_KI 0.11f
//#define M2006_MOTOR_SPEED_PID_KD 1.0f
#define M2006_MOTOR_SPEED_PID_MAX_OUT 10000.0f        //最大输出电流
#define M2006_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//M2006电机角度环PID
#define M2006_MOTOR_POSITION_PID_KP 1500.0f
#define M2006_MOTOR_POSITION_PID_KI 0.2f
#define M2006_MOTOR_POSITION_PID_KD 0.0f
#define M2006_MOTOR_POSITION_PID_MAX_OUT 3000.0f
#define M2006_MOTOR_POSITION_PID_MAX_IOUT 400.0f


typedef struct
{
    motor_measure_t *enginer_motor_measure;
		int16_t  target_speed;//目标速度
		float target_angle;//目标角度
		float angle;//累积角度
		int16_t give_current;
} enginer_motor_t;


typedef struct
{
    enginer_motor_t motor;

	  pid_type_def motor_speed_pid;             //底盘电机速度pid
		pid_type_def motor_angle_pid;              //底盘跟随角度pid
} enginer_control_t;



//初始化函数 
//参数：(电机控制结构体变量，速度跟随pid，位置跟随pid，速度环最大输出，速度环最大积分输出，角度环最大输出，角度环最大pid输出, 电机ID)
void enginer_init(enginer_control_t *enginer_move_init, fp32 motor_speed_pid[3], fp32 motor_positon_pid[3], fp32 SPEED_MAX_OUT, \
	fp32 SPEED_PID_MAX_IOUT, fp32 POSITION_PID_MAX_OUT, fp32 POSITION_PID_MAX_IOUT, uint8_t ID);


//实现pid闭环
//参数：(电机控制结构体，控制模式，电机角度比)    控制模式: 1-->角度环控制   0-->速度环控制
void PIDrealize(enginer_control_t *enginer_move, uint8_t model, fp32 MOTOR_ECD_TO_ANGLE);
#endif
