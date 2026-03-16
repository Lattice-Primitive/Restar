#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

#define CHASSIS_CAN hcan1
#define GIMBAL_CAN 	hcan2

//#define P_MIN		-12.5f
//#define P_MAX		12.5f
//#define V_MIN		-45.0f
//#define V_MAX		45.0f
//#define KP_MIN	0.0f
//#define KP_MAX	500.0f
//#define KD_MIN 	0.0f
//#define KD_MAX	5.0f
//#define T_MIN		-54.0f
//#define T_MAX		54.0f


#define IMU_External_ID  0x0230       //外置IMU接入ID



/* CAN send and receive ID */
typedef enum
{
    CAN_ALL_ID1 = 0x200,
    CAN_ID0 = 0x201,
    CAN_ID1 = 0x202,
    CAN_ID2 = 0x203,
    CAN_ID3 = 0x204,
	
	CAN_ALL_ID2 = 0x1FF,
    CAN_ID4 = 0x205,
    CAN_ID5 = 0x206,
    CAN_ID6 = 0x207,
	CAN_ID7 = 0x208,
} can_msg_id_e;

//rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
	int16_t ecd_count;//电机轴圈数累积
} motor_measure_t;

//IMU设备数据
typedef struct
{
	float INS_angle[3];         //欧拉角
	float INS_accel[3];        //加速度
	float INS_gyro[3];         //角速度
	float INS_quat[4];         //四元数
	float temp;                //温度
}IMU_DATA;         





/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
//外置IMU设置ID
void IMU_External_Set_ID(uint16_t ID);
//外置IMU设置控制温度
void IMU_External_Set_Temp(float temp);
//外置IMU设置发送数据
void IMU_External_Set_Data(_Bool INS_Angle, _Bool ACCEL, _Bool GYRO, _Bool Quat, _Bool Temp);
//外置IMU零漂校准
void IMU_External_Cail(void);
//发送外置IMU姿态模块的CAN包
void CAN_Send_IMU_Data(uint16_t IMU_ID, uint8_t CAN_TX_BUF[]);

//返回外置IMU数据指针
IMU_DATA *get_imu_external_point(void);
//返回CAN1电机数据指针
motor_measure_t *get_motor_point_can1(uint8_t number);
//返回CAN2电机数据指针
motor_measure_t *get_motor_point_can2(uint8_t number);

//底盘电机快速校准ID
void CAN_cmd_chassis_reset_ID(void);
//CAN1发送0~3号电机数据
void CAN1_cmd_0_3(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
//CAN1发送4~7号电机数据
void CAN1_cmd_4_7(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
////CAN2发送0~3号电机数据
//void CAN2_cmd_0_3(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
////CAN2发送4~7号电机数据
//void CAN2_cmd_4_7(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);





#endif
