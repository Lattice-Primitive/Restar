#include "referee_usart_task.h"
#include "referee.h"
#include "bsp_usart.h"
#include "usart.h"
#include "referee.h"


uint8_t usart6_buf[2][USART_RX_BUF_LENGHT];

//裁判系统串口初始化
void referee_uart_init(void){
	
	usart6_init(usart6_buf[0], usart6_buf[1], USART_RX_BUF_LENGHT);
	
}




//用户定义的串口中断    需要放到HAL库的串口中断函数里
void USER_USART6_IRQHandler(void)
{
    if(USART6->SR & UART_FLAG_IDLE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart6);   //清除空闲标志
        static uint16_t this_time_rx_len = 0;			//长度
        if ((huart6.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(huart6.hdmarx);				//禁用DMA
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);   //数据长度
            __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);			//设置传输数据量
            huart6.hdmarx->Instance->CR |= DMA_SxCR_CT;				//设置缓冲区1
            __HAL_DMA_ENABLE(huart6.hdmarx);			//使能DMA
//            detect_hook(REFEREE_TOE);
					referee_data_updata(usart6_buf[0],this_time_rx_len);
        }
        else
        {
            __HAL_DMA_DISABLE(huart6.hdmarx);				//禁用DMA
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);			//数据长度
            __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);			//设置传输数据量
            huart6.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);				//设置缓冲区0
            __HAL_DMA_ENABLE(huart6.hdmarx);			//使能DMA
//            detect_hook(REFEREE_TOE);
						referee_data_updata(usart6_buf[1],this_time_rx_len);
        }
    }
}








