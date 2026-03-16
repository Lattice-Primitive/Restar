#include "fsus.h"
#include "drv_fsus.h"
#include "stm32f4xx_hal.h"
#include "drv_delay.h"

#define USART_RECV_BUF_SIZE 2048
#define USART_SEND_BUF_SIZE 2048

extern UART_HandleTypeDef huart8;
uint8_t rx_data;
FSUS_t FSUS1;

void FSUS1_Write(uint8_t data)
{
    uint8_t tx_data=data;
    HAL_UART_Transmit(&huart8,&tx_data,1,1000);
}

uint8_t FSUS1_Read(void)
{
    RingBuffer_Push(FSUS1.recvBuf,rx_data);
    return HAL_UART_Receive_IT(&huart8,&rx_data,1);
}

FSUS_Operation_t fsus_opt=
{
    .FSUS_Delay_Ms=HAL_Delay,
    .FSUS_CountdownIsTimeout=Delay_CountdownIsTimeout,
    .FSUS_CountdownCancel=Delay_CountdownCancel,
    .FSUS_CountdownBegin=Delay_CountdownBegin,
    .FSUS_UART_ReadData=FSUS1_Read,
    .FSUS_UART_WriteData=FSUS1_Write,
};

uint8_t SendBuf1[USART_SEND_BUF_SIZE + 1];
uint8_t RecvBuf1[USART_RECV_BUF_SIZE + 1];
RingBufferTypeDef FSUS1_SendBuffer;
RingBufferTypeDef FSUS1_RecvBuffer;

void FSUS_Init(void)
{
    RingBuffer_Init(&FSUS1_SendBuffer, USART_SEND_BUF_SIZE, SendBuf1);
    RingBuffer_Init(&FSUS1_RecvBuffer, USART_RECV_BUF_SIZE, RecvBuf1);
    FSUS1.sendBuf=&FSUS1_SendBuffer;
    FSUS1.recvBuf=&FSUS1_RecvBuffer;
    FSUS1.opt=&fsus_opt;
    HAL_UART_Receive_IT(&huart8,&rx_data,1);
}

void FSUS_Run(void)
{
    // 舵机控制相关的参数
	// 舵机的ID号
	uint8_t servo_id = 0;
	// 舵机的目标角度
	// 舵机角度在-135度到135度之间, 精确到小数点后一位
	float angle = 0;
	// 时间间隔ms
	// 可以尝试修改设置更小的时间间隔，例如500ms
	uint16_t interval;
	// 目标转速
	float velocity;
	// 加速时间
	uint16_t t_acc;
	// 减速时间
	uint16_t t_dec;
	// 舵机执行功率 mV 默认为0
	uint16_t power = 0;
	// 设置舵机角度的时候, 是否为阻塞式
	// 0:不等待 1:等待舵机旋转到特定的位置;
	uint8_t wait = 1;
	// 读取的角度
	float angle_read;

	//FSUS_DampingMode(&FSUS1,servo_id,0);
	while (1)
	{
		// 控制舵机角度
		angle = 90.0;
		interval = 2000;
		FSUS_SetServoAngle(&FSUS1, servo_id, angle, interval, power, wait);
		FSUS_QueryServoAngle(&FSUS1, servo_id, &angle_read);

		// 等待2s
		HAL_Delay(2000);

		// 控制舵机角度 + 指定时间
		
		angle = 0.0f;
		interval = 1000;
		t_acc = 100;
		t_dec = 150;
		FSUS_SetServoAngleByInterval(&FSUS1, servo_id, angle, interval, t_acc, t_dec, power, wait);
		FSUS_QueryServoAngle(&FSUS1, servo_id, &angle_read);
		
		// 等待2s
		HAL_Delay(2000);

		// 控制舵机角度 + 指定转速
		angle = -90.0f;
		velocity = 200.0f;
		t_acc = 100;
		t_dec = 150;
		FSUS_SetServoAngleByVelocity(&FSUS1, servo_id, angle, velocity, t_acc, t_dec, power, wait);
		FSUS_QueryServoAngle(&FSUS1, servo_id, &angle_read);
		HAL_Delay(2000);
		/* FSUS_QueryServoAngle(&FSUS1, servo_id, &angle_read);
		HAL_Delay(1000); */
	}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{   
    if(huart==&huart8)
    {
        FSUS1.opt->FSUS_UART_ReadData();
    }
}
