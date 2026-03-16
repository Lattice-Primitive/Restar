
#ifndef INS_Task_H
#define INS_Task_H
#include "struct_typedef.h"
#include "CAN_receive.h"




#define TEMPERATURE_PID_KP 1600.0f //温度控制PID的kp
#define TEMPERATURE_PID_KI 0.2f    //温度控制PID的ki
#define TEMPERATURE_PID_KD 0.0f    //温度控制PID的kd
#define TEMPERATURE_PID_MAX_OUT   4500.0f //温度控制PID的max_out
#define TEMPERATURE_PID_MAX_IOUT 4400.0f  //温度控制PID的max_iout


#define MPU6500_TEMP_PWM_MAX 5000 //mpu6500控制温度的设置TIM的重载值，即给PWM最大为 MPU6500_TEMP_PWM_MAX - 1


/**
  * @brief          陀螺仪校准任务
  * @param[in]      none
	* @retval         none
  */
void IMU_cali_task(void);
/**
  * @brief          返回内置IMU数据指针
  * @param[in]      nonoe
  * @retval         none
  */
IMU_DATA *get_imu_inside_point(void);


#endif
