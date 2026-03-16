#include "drv_wt61c.h"

volatile static uint8_t ucRxBuffer[11];//暂存缓冲
volatile static uint8_t ucRxCnt = 0;//暂存缓冲计数

/*WT61C暂存数据*/
typedef struct 
{
    volatile short accel[4];/*accel_x accel_y accel_z temp*/
    volatile short gyro[4];/*gyro_x gyro_y gyro_z temp*/
    volatile short angle[4];/*angle_x angle_y angle_z temp*/
}WT61C_Data_t;

WT61C_Data_t wt61c_data;

#ifdef USE_HAL_DRIVER /*HAL库*/
#include "stm32f4xx_ll_usart.h"

static USART_TypeDef *wt61c_uart=NULL;

void WT61C_Init(void *wt61c_uart, WT61C_BaudRate_t baud_rate)
{
	wt61c_uart=(USART_TypeDef*)wt61c_uart;
	WT61C_SendCmd(ZANGLE_INIT);//使Z轴角度归零
	WT61C_SendCmd(ACCEL_CORRECT);//校准加速度零偏
	if(WT61C_BAUD_9600==baud_rate)//如果设置波特率为9600
	{
		WT61C_SendCmd(BAUD_9600);
	}
	else if(WT61C_BAUD_115200==baud_rate)
	{
		WT61C_SendCmd(BAUD_115200);
	}
}

void WT61C_SendCmd(WT61C_Cmd_t cmd)
{
	if(wt61c_uart==NULL)
	{
		return;
	}
	LL_USART_TransmitData8(wt61c_uart,0xff);
	while(LL_USART_IsActiveFlag_TXE(wt61c_uart));//等待数据移入shift register
	LL_USART_TransmitData8(wt61c_uart,0xaa);
	while(LL_USART_IsActiveFlag_TXE(wt61c_uart));
	LL_USART_TransmitData8(wt61c_uart,cmd);
	while(LL_USART_IsActiveFlag_TXE(wt61c_uart));
}

#endif /*USE_HAL_DRIVER*/


void WT61C_GetData_Callback(uint8_t get_data)
{
    ucRxBuffer[ucRxCnt++] = get_data; //将收到的数据存入缓冲区中
	if (ucRxBuffer[0] != 0x55)		//数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt = 0;
		return;
	}
	if (ucRxCnt < 11)
	{
		return;
	} //数据不满11个，则返回
	else
	{
		switch (ucRxBuffer[1]) //判断数据是哪种数据，然后将其拷贝到对应的结构体中
		{
		case 0x51://加速度
			memcpy(wt61c_data.accel, &ucRxBuffer[2], 8);//将数据拷贝到结构体相应位置中
			break;

		case 0x52://角速度
			memcpy(wt61c_data.gyro, &ucRxBuffer[2], 8);
			break;

		case 0x53://角度
			memcpy(&wt61c_data.angle, &ucRxBuffer[2], 8);
			break;
		}
		ucRxCnt = 0; //清空缓存区
	}
}

