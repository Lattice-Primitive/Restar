#include "enginer_task.h"
#include "main.h"

enginer_control_t enginer_move[8];
extern motor_measure_t motor_can1_data[8];

//返回CAN1电机数据指针
 motor_measure_t *get_motor_point(uint8_t number)
{
	if( number < 8 )
    return &motor_can1_data[number];
	else
		return NULL;
}

//初始化函数 
//参数：(电机控制结构体变量，速度跟随pid，位置跟随pid，速度环最大输出，速度环最大积分输出，角度环最大输出，角度环最大积分输出, 电机ID)
void enginer_init(enginer_control_t *enginer_move_init, fp32 motor_speed_pid[3], fp32 motor_positon_pid[3], fp32 SPEED_MAX_OUT, \
	fp32 SPEED_PID_MAX_IOUT, fp32 POSITION_PID_MAX_OUT, fp32 POSITION_PID_MAX_IOUT, uint8_t ID) 
{
	//获取电机指针
	enginer_move_init->motor.enginer_motor_measure = get_motor_point(ID);
	//初始化PID控制参数
	PID_init(&enginer_move_init->motor_speed_pid, PID_POSITION, motor_speed_pid, SPEED_MAX_OUT, SPEED_PID_MAX_IOUT);
	PID_init(&enginer_move_init->motor_angle_pid, PID_POSITION, motor_positon_pid, POSITION_PID_MAX_OUT, POSITION_PID_MAX_IOUT);
	//清除PID数据参数
	PID_clear(&enginer_move_init->motor_speed_pid);
	PID_clear(&enginer_move_init->motor_angle_pid);
}


//实现pid闭环
//参数：(电机控制结构体，控制模式，电机角度比)    控制模式: 1-->角度环控制   0-->速度环控制
void PIDrealize(enginer_control_t *enginer_move, uint8_t model, fp32 MOTOR_ECD_TO_ANGLE)
{
	if(model){     //角度环模式
		//更新角度
		enginer_move->motor.angle = (enginer_move->motor.enginer_motor_measure->ecd_count*ECD_RANGE +enginer_move->motor.enginer_motor_measure->ecd)*MOTOR_ECD_TO_ANGLE;
		//角度环解算---->输出跟随速度
		enginer_move->motor.target_speed = PID_calc(&enginer_move->motor_angle_pid,enginer_move->motor.angle,enginer_move->motor.target_angle);
		//速度环解算---->输出跟随电流
		enginer_move->motor.give_current = PID_calc(&enginer_move->motor_speed_pid,enginer_move->motor.enginer_motor_measure->speed_rpm,enginer_move->motor.target_speed);
	}
	else{
		//速度环模式，直接速度环解算---->输出跟随电流
		enginer_move->motor.give_current = PID_calc(&enginer_move->motor_speed_pid,enginer_move->motor.enginer_motor_measure->speed_rpm,enginer_move->motor.target_speed);
	}
}





