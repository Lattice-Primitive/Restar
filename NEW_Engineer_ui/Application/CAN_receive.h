#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

#define CHASSIS_CAN hcan1
#define GIMBAL_CAN 	hcan2

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


extern motor_measure_t motor_can1_data[8];    	//can1电机返回数据
extern motor_measure_t motor_can2_data[8];			//can2电机返回数据


/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */


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
//CAN2发送0~3号电机数据
void CAN2_cmd_0_3(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
//CAN2发送4~7号电机数据
void CAN2_cmd_4_7(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

#endif
