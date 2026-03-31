#include "enginer_task.h"
#include "main.h"
#include "CAN_receive.h"

#include "cmsis_os.h"

enginer_control_t enginer_move_can1[8];     		//CAN1需要控制的电机结构体
enginer_control_t enginer_move_can2[8];					//CAN2需要控制的电机结构体
extern motor_measure_t motor_can1_data[8];     	//CAN1上电机回传数据
extern motor_measure_t motor_can2_data[8];			//CAN2上电机回传数据


/**
  * @brief          映射函数
  * @param[in]      输入值
	* @param[in]			输入最小值
	* @param[in]			输入最大值
	* @param[in]			输出最小值
	* @param[in]			输出最大值
  * @retval         映射值
  */
fp32 map(fp32 x, fp32 in_min, fp32 in_max, fp32 out_min, fp32 out_max)
{
	if(x <= in_min)
		return out_min;
	else if(x > in_max)
		return out_max;
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
} 

/**
  * @brief          电机控制PID初始化函数 
  * @param[in]      电机控制结构体
	* @param[in]			所使用的CAN口    CAN1或CAN2
	* @param[in]			电机ID      0~8
	* @param[in]			速度环PID
	* @param[in]			速度环最大输出
	* @param[in]			速度环最大积分输出
	* @param[in]			位置环PID
	* @param[in]			位置环最大输出
	* @param[in]			位置环最大积分输出
  * @retval         none
  */
void enginer_init(enginer_control_t *enginer_move_init, uint8_t MOTOR_ID, uint8_t CAN, fp32 SPEED_PID[3], fp32 SPEED_MAX_OUT, \
	fp32 SPEED_PID_MAX_IOUT, fp32 POSITION_PID[3], fp32 POSITION_PID_MAX_OUT, fp32 POSITION_PID_MAX_IOUT) 
{
	//获取电机指针
	if(1 == CAN)        //CAN1的电机
		enginer_move_init->motor.enginer_motor_measure = get_motor_point_can1(MOTOR_ID);
	else if(2 == CAN)		//CAN2的电机
		enginer_move_init->motor.enginer_motor_measure = get_motor_point_can2(MOTOR_ID);
	else
		return;
	//初始化PID控制参数
	PID_init(&enginer_move_init->motor_speed_pid, PID_POSITION, SPEED_PID, SPEED_MAX_OUT, SPEED_PID_MAX_IOUT);
	PID_init(&enginer_move_init->motor_angle_pid, PID_POSITION, POSITION_PID, POSITION_PID_MAX_OUT, POSITION_PID_MAX_IOUT);
	//清除PID数据参数
	PID_clear(&enginer_move_init->motor_speed_pid);
	PID_clear(&enginer_move_init->motor_angle_pid);
}

/**
  * @brief         	PID闭环实现函数
  * @param[in]      电机控制结构体指针
	* @param[in]			控制模式     1：位置环控制 POSITION_RING_CONTROL  0：速度环控制 SPEED_RING_CONTROL
	* @param[in]			电机码盘角度换算比
  * @retval         none
  */
void PIDrealize(enginer_control_t *enginer_move, uint8_t MOTOR_MODEL, fp32 MOTOR_ECD_TO_ANGLE)
{
	if(POSITION_RING_CONTROL == MOTOR_MODEL){     //位置环模式
		//更新角度
		enginer_move->motor.angle = (enginer_move->motor.enginer_motor_measure->ecd_count*ECD_RANGE +enginer_move->motor.enginer_motor_measure->ecd)*MOTOR_ECD_TO_ANGLE;
		//位置环解算---->输出跟随速度
		enginer_move->motor.target_speed = PID_calc(&enginer_move->motor_angle_pid,enginer_move->motor.angle,enginer_move->motor.target_angle);
		//速度环解算---->输出跟随电流
		enginer_move->motor.give_current = PID_calc(&enginer_move->motor_speed_pid,enginer_move->motor.enginer_motor_measure->speed_rpm,enginer_move->motor.target_speed);
	}
	else if(SPEED_RING_CONTROL == MOTOR_MODEL){   //速度环模式
		//速度环模式，直接速度环解算---->输出跟随电流
		enginer_move->motor.give_current = PID_calc(&enginer_move->motor_speed_pid,enginer_move->motor.enginer_motor_measure->speed_rpm,enginer_move->motor.target_speed);
	}
}

void Chassis_Spd(const chassis_speed_t speed)
{
    //because the gimbal is in front of chassis, when chassis rotates, wheel 0 and wheel 1 should be slower and wheel 2 and wheel 3 should be faster
    //旋转的时候， 由于云台靠前，所以是前面两轮 0 ，1 旋转的速度变慢， 后面两轮 2,3 旋转的速度变快
		enginer_move_can1[0].motor.target_speed = (-speed.vx_set + speed.vy_set + speed.wz_set);
		enginer_move_can1[1].motor.target_speed = (speed.vx_set  + speed.vy_set + speed.wz_set);
	  enginer_move_can1[2].motor.target_speed = (speed.vx_set  - speed.vy_set + speed.wz_set);
	  enginer_move_can1[3].motor.target_speed = (-speed.vx_set - speed.vy_set + speed.wz_set);
}








