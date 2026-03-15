/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

#define CHASSIS_CAN hcan

/* CAN send and receive ID */
typedef enum
{
    CAN_ALL_ID1 = 0x200,
    CAN_ID1 = 0x201,
    CAN_ID2 = 0x202,
    CAN_ID3 = 0x203,
    CAN_ID4 = 0x204,
	
	CAN_ALL_ID2 = 0x1FF,
    CAN_ID5 = 0x205,
    CAN_ID6 = 0x206,
    CAN_ID7 = 0x207,
	CAN_ID8 = 0x208,
    

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




/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
extern void CAN_cmd_chassis_reset_ID(void);


extern void CAN1_cmd_0_3(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void CAN1_cmd_4_7(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void CAN2_cmd_0_3(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void CAN2_cmd_4_7(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

extern motor_measure_t  *get_motor_point(uint8_t number);
#endif
