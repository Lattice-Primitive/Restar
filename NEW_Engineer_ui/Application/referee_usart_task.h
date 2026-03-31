#ifndef REFEREE_USART_TASK_H
#define REFEREE_USART_TASK_H


#include "main.h"


#define USART_RX_BUF_LENGHT     512




extern uint8_t usart6_buf[2][USART_RX_BUF_LENGHT];














void referee_uart_init(void);
void USER_USART6_IRQHandler(void);






#endif

