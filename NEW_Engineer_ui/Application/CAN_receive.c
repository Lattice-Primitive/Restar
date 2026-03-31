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
  *	 V2.0.0			July-3-2023			R&A战队					2. update
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "CAN_receive.h"
#include "main.h"

#include "detect_task.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

motor_measure_t motor_can1_data[8];    	//can1电机返回数据
motor_measure_t motor_can2_data[8];			//can2电机返回数据
		
extern int x_distance,z_distance;			//x和z移动距离
		
static CAN_TxHeaderTypeDef  can_tx_message;
static uint8_t              can_send_data[8];

/**
  * @brief          将float类型数据按位存到uint8_t类型的变量里
  * @param[in]      uint8_t类型指针
  * @param[in]      float类型指针
  * @param[in]      存入数据的字节数  1字节8位   float类型为32位
	* @retval         none
  */
//static void float_to_uint8_t(uint8_t *p, float *a, uint16_t len){
//	uint16_t i;
//	uint8_t *q;
//	q = (uint8_t *)a;
//	for(i=0;i<len;i++){
//		*p = *q;
//		p++;
//		q++;
//	}
//}

/**
  * @brief          将uint8_t类型数据按位存到float类型的变量里
  * @param[in]      float类型指针
  * @param[in]      uint8_t类型指针
  * @param[in]      存入数据的字节数  1字节8位   uint8_t类型为32位
	* @retval         none
  */
//static void uint8_t_to_float(float *p, uint8_t *a, uint16_t len){
//	uint16_t i;
//	uint8_t *q;
//	q = (uint8_t *)p;
//	for(i=0;i<len;i++){
//		*q = *a;
//		a++;
//		q++;
//	}
//}


/**
  * @brief          返回CAN1电机数据指针
  * @param[in]      第n个电机    n:0~7
  * @retval         第n个电机的数据首地址
  */
motor_measure_t *get_motor_point_can1(uint8_t number)
{
	if( number < 8 )
    return &motor_can1_data[number];
	else
		return NULL;
}

/**
  * @brief          返回CAN2电机数据指针
  * @param[in]      第n个电机    n:0~7
  * @retval         第n个电机的数据首地址
  */
motor_measure_t *get_motor_point_can2(uint8_t number)
{
	if( number < 8 )
    return &motor_can2_data[number];
	else
		return NULL;
}


/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];
	if(hcan->Instance == CAN1)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
	  if (rx_header.StdId <= 0x07)			
		{	
			//达妙电机反馈数据
			FEEDBACK_DATA_DM_J8009(rx_header.StdId , rx_data);
		}
	}
	if(hcan->Instance == CAN2)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
		if (rx_header.StdId <= 0x05)			
		{	
			//达妙电机反馈数据
			FEEDBACK_DATA_DM_J8009(rx_header.StdId , rx_data);
		}
	}
	if(hcan->Instance == CAN1)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
		switch (rx_header.StdId)
		{
			case CAN_ID0:
			case CAN_ID1:
			case CAN_ID2:
			case CAN_ID3:
			case CAN_ID4:
			case CAN_ID5:
			case CAN_ID6:
			case CAN_ID7:
			{
				static uint8_t i = 0;
				//get motor id
				i = rx_header.StdId - CAN_ID0;
				get_motor_measure(&motor_can1_data[i], rx_data);
				//记录电机圈数，位置控制时需要用到
				if (motor_can1_data[i].ecd - motor_can1_data[i].last_ecd> 4096){
						motor_can1_data[i].ecd_count--;
				}
				else if (motor_can1_data[i].ecd - motor_can1_data[i].last_ecd<-4096){
						motor_can1_data[i].ecd_count++;
				}
				//防止使用速度环时圈数数据溢出
				if(motor_can1_data[i].ecd_count > 30000 || motor_can1_data[i].ecd_count < -30000)
					motor_can1_data[i].ecd_count=0;
				//记录电机数据接收时间
				if(i<=3)
					detect_hook(CHASSIS_MOTOR1_TOE + i);
				//x和z移动距离
				x_distance = (uint32_t)(motor_can1_data[4].ecd_count * 8191) + (uint32_t)(motor_can1_data[4].ecd);
				z_distance = ((uint32_t)(-motor_can1_data[6].ecd_count * 8191) - (uint32_t)(motor_can1_data[6].ecd));
				break;
				
				
			}
		}
	}
	if(hcan->Instance == CAN2)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
		switch (rx_header.StdId)
		{
			case CAN_ID0:
			case CAN_ID1:
			case CAN_ID2:
			case CAN_ID3:
			case CAN_ID4:
			case CAN_ID5:
			case CAN_ID6:
			case CAN_ID7:
			{
				static uint8_t i = 0;
				//get motor id
				i = rx_header.StdId - CAN_ID0;
				get_motor_measure(&motor_can2_data[i], rx_data);
				//记录电机圈数，位置控制时需要用到
				if (motor_can2_data[i].ecd - motor_can2_data[i].last_ecd> 4096){
						motor_can2_data[i].ecd_count--;
				}
				else if (motor_can2_data[i].ecd - motor_can2_data[i].last_ecd<-4096){
						motor_can2_data[i].ecd_count++;
				}
				//防止使用速度环时圈数数据溢出
				if(motor_can2_data[i].ecd_count > 30000 || motor_can2_data[i].ecd_count < -30000)
					motor_can2_data[i].ecd_count=0;
				break;
			}
		}
	}
}

/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    can_tx_message.StdId = 0x700;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = 0;
    can_send_data[1] = 0;
    can_send_data[2] = 0;
    can_send_data[3] = 0;
    can_send_data[4] = 0;
    can_send_data[5] = 0;
    can_send_data[6] = 0;
    can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &can_tx_message, can_send_data, &send_mail_box);
}



//CAN1发送电流给ID为0~3号的电机
void CAN1_cmd_0_3(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    can_tx_message.StdId = CAN_ALL_ID1;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = motor1 >> 8;
    can_send_data[1] = motor1;
    can_send_data[2] = motor2 >> 8;
    can_send_data[3] = motor2;
    can_send_data[4] = motor3 >> 8;
    can_send_data[5] = motor3;
    can_send_data[6] = motor4 >> 8;
    can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan1, &can_tx_message, can_send_data, &send_mail_box);
}

//CAN1发送电流给ID为4~7号的电机
void CAN1_cmd_4_7(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    can_tx_message.StdId = CAN_ALL_ID2;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = motor1 >> 8;
    can_send_data[1] = motor1;
    can_send_data[2] = motor2 >> 8;
    can_send_data[3] = motor2;
    can_send_data[4] = motor3 >> 8;
    can_send_data[5] = motor3;
    can_send_data[6] = motor4 >> 8;
    can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan1, &can_tx_message, can_send_data, &send_mail_box);
}



//CAN2发送电流给ID为0~3号的电机
void CAN2_cmd_0_3(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    can_tx_message.StdId = CAN_ALL_ID1;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = motor1 >> 8;
    can_send_data[1] = motor1;
    can_send_data[2] = motor2 >> 8;
    can_send_data[3] = motor2;
    can_send_data[4] = motor3 >> 8;
    can_send_data[5] = motor3;
    can_send_data[6] = motor4 >> 8;
    can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan2, &can_tx_message, can_send_data, &send_mail_box);
}

//CAN2发送电流给ID为4~7号的电机
void CAN2_cmd_4_7(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    can_tx_message.StdId = CAN_ALL_ID2;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = motor1 >> 8;
    can_send_data[1] = motor1;
    can_send_data[2] = motor2 >> 8;
    can_send_data[3] = motor2;
    can_send_data[4] = motor3 >> 8;
    can_send_data[5] = motor3;
    can_send_data[6] = motor4 >> 8;
    can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan2, &can_tx_message, can_send_data, &send_mail_box);
}















