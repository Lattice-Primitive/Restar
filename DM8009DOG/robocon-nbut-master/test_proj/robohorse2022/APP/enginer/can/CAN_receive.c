/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0         Dec-26-2018       RM                    1. done
  *	 V2.0.0			July-3-2023			R&A战队				2. update
  *	 V3.0.0			July-12-2024		R&A战队				3. update

  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
  

#include "CAN_receive.h"
#include "main.h"
#include "detect_task.h"
#include "dm8009_drv.h"

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
		
IMU_DATA IMU_External_Data;					//外置接入的IMU数据
uint16_t IMU_Data_Top = 0;   //数据头
uint8_t IMU_BUF[8];          //姿态模块命令数据的缓存数组


CAN_TxHeaderTypeDef  can_tx_message;
uint8_t              can_send_data[8];

    
    
/**
  * @brief          将float类型数据按位存到uint8_t类型的变量里
  * @param[in]      uint8_t类型指针
  * @param[in]      float类型指针
  * @param[in]      存入数据的字节数  1字节8位   float类型为32位
	* @retval         none
  */
static void float_to_uint8_t(uint8_t *p, float *a, uint16_t len){
	uint16_t i;
	uint8_t *q;
	q = (uint8_t *)a;
	for(i=0;i<len;i++){
		*p = *q;
		p++;
		q++;
	}
}



/**
  * @brief          将uint8_t类型数据按位存到float类型的变量里
  * @param[in]      float类型指针
  * @param[in]      uint8_t类型指针
  * @param[in]      存入数据的字节数  1字节8位   uint8_t类型为32位
	* @retval         none
  */
static void uint8_t_to_float(float *p, uint8_t *a, uint16_t len){
	uint16_t i;
	uint8_t *q;
	q = (uint8_t *)p;
	for(i=0;i<len;i++){
		*q = *a;
		a++;
		q++;
	}
}




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
  * @brief          返回外置IMU数据指针
  * @param[in]      nonoe
  * @retval         none
  */
IMU_DATA *get_imu_external_point(void)
{
    return &IMU_External_Data;
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
	
}




/**
  * @brief          外置IMU设置ID
  * @param[in]      ID号  
  * @retval         none
  */
void IMU_External_Set_ID(uint16_t ID){
	uint8_t tx_buf[8];
	tx_buf[0] = 0x00;    
	tx_buf[1] = 0xFF;    //ID头位0xFF时，对所有ID头模块均可更改ID
	tx_buf[2] = 0x01;    //指令
	tx_buf[3] = 0x00;
	tx_buf[4] = 0x00;
	tx_buf[5] = ID>>8;
	tx_buf[6] = ID;     //需要修改的ID
	tx_buf[7] = tx_buf[2] + tx_buf[3] + tx_buf[4] + tx_buf[5] + tx_buf[6];     //校验和
	CAN_Send_IMU_Data(0x00FF,tx_buf);					//CAN发送
}



/**
  * @brief          外置IMU设置控制温度
  * @param[in]      温度  30~50℃
  * @retval         none
  */
void IMU_External_Set_Temp(float temp){
	uint8_t tx_buf[8];
	tx_buf[0] = IMU_External_ID >> 8;    
	tx_buf[1] = (uint8_t)IMU_External_ID;    //ID头
	tx_buf[2] = 0x02;      //指令
	float_to_uint8_t(tx_buf,&temp,4);			//温度转存到发送数组
	tx_buf[7] = tx_buf[2] + tx_buf[3] + tx_buf[4] + tx_buf[5] + tx_buf[6];     //校验和
	CAN_Send_IMU_Data(IMU_External_ID,tx_buf);    	//CAN发送
}



/**
  * @brief          外置IMU设置发送数据
  * @param[in]      姿态角   	1开启0关闭
	* @param[in]      加速度		1开启0关闭
	* @param[in]      角速度		1开启0关闭
	* @param[in]      四元数		1开启0关闭
	* @param[in]      温度			1开启0关闭
  * @retval         none
  */
void IMU_External_Set_Data(_Bool INS_Angle, _Bool ACCEL, _Bool GYRO, _Bool Quat, _Bool Temp){
	uint8_t tx_buf[8];
	tx_buf[0] = IMU_External_ID >> 8;    
	tx_buf[1] = (uint8_t)IMU_External_ID;    //ID头
	tx_buf[2] = 0x03;      //指令
	tx_buf[3] = INS_Angle | ACCEL<<1 | GYRO<<2 | Quat<<3 | Temp<<4;    //数据信息
	tx_buf[4] = 0x00;
	tx_buf[5] = 0x00;
	tx_buf[6] = 0x00;
	tx_buf[7] = tx_buf[2] + tx_buf[3] + tx_buf[4] + tx_buf[5] + tx_buf[6];     //校验和
	CAN_Send_IMU_Data(IMU_External_ID,tx_buf);    	//CAN发送
}



/**
  * @brief          外置IMU零漂校准
  * @param[in]      none
  * @retval         none
  */
void IMU_External_Cail(void){
	uint8_t tx_buf[8];
	tx_buf[0] = IMU_External_ID >> 8;    
	tx_buf[1] = (uint8_t)IMU_External_ID;    //ID头
	tx_buf[2] = 0x04;      //指令
	tx_buf[3] = 0x00;
	tx_buf[4] = 0x00;
	tx_buf[5] = 0x00;
	tx_buf[6] = 0x00;
	tx_buf[7] = tx_buf[2] + tx_buf[3] + tx_buf[4] + tx_buf[5] + tx_buf[6];     //校验和
	CAN_Send_IMU_Data(IMU_External_ID,tx_buf);    	//CAN发送
}




/**
  * @brief          发送外置IMU姿态模块的CAN包
	* @param[in]      姿态模块传感器ID头
  * @param[in]      发送的数据数组
  * @retval         none
  */
void CAN_Send_IMU_Data(uint16_t IMU_ID, uint8_t CAN_TX_BUF[])
{
    uint32_t send_mail_box;
    can_tx_message.StdId = IMU_ID;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = CAN_TX_BUF[0];
    can_send_data[1] = CAN_TX_BUF[1];
    can_send_data[2] = CAN_TX_BUF[2];
    can_send_data[3] = CAN_TX_BUF[3];
    can_send_data[4] = CAN_TX_BUF[4];
    can_send_data[5] = CAN_TX_BUF[5];
    can_send_data[6] = CAN_TX_BUF[6];
    can_send_data[7] = CAN_TX_BUF[7];
		
    HAL_CAN_AddTxMessage(&hcan1, &can_tx_message, can_send_data, &send_mail_box);
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






