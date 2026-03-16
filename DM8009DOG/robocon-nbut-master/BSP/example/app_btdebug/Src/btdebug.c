#include "btdebug.h"
#include "app_btdebug.h"
#include "main.h"

extern UART_HandleTypeDef huart7;

void BT_RxByte(uint8_t *pData, uint32_t Size);
void BT_TxByte(uint8_t *pData, uint32_t Size);

BTDebug_Operate_t btdebug_opt=
{
    .ReceiveBytes=BT_RxByte,
    .TransmitBytes=BT_TxByte,
};

uint8_t led_status[2]={0x00,0x22};
uint8_t rx_data[2];

void BTDebug_Run()
{
    HAL_GPIO_WritePin(GPIOG,((uint16_t)led_status[0])<<1,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOG,~(((uint16_t)led_status[0])<<1),GPIO_PIN_SET);
    BTDebug_TransmitPackage(&btdebug_opt,led_status,2);
    HAL_Delay(50);
    if(BTDebug_ReceivePackage(&btdebug_opt,rx_data,2)==BTDEBUG_TRUE)
    {
        led_status[0]=rx_data[0];
        led_status[1]=rx_data[1];
    }
    HAL_Delay(50);
}

void BT_RxByte(uint8_t *pData, uint32_t Size)
{
    HAL_UART_Receive(&huart7,pData,Size,100);
}

void BT_TxByte(uint8_t *pData, uint32_t Size)
{
    HAL_UART_Transmit(&huart7,pData,Size,100);
}
