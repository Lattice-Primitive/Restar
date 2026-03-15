#include "remote_control.h"
#include "main.h"
#include "string.h"


//遥控器出错数据上限
#define RC_CHANNAL_ERROR_VALUE 700

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

//遥控器控制变量
RC_ctrl_t rc_ctrl;
//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
uint8_t sbus_rx_buf[SBUS_RX_BUF_NUM];


//判断遥控器数据是否出错，
uint8_t RC_data_is_error(void)
{
    //使用了go to语句 方便出错统一处理遥控器变量数据归零
    if (RC_abs(rc_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[0] == 0)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[1] == 0)
    {
        goto error;
    }
    return 0;

error:
    rc_ctrl.rc.ch[0] = 0;
    rc_ctrl.rc.ch[1] = 0;
    rc_ctrl.rc.ch[2] = 0;
    rc_ctrl.rc.ch[3] = 0;
    rc_ctrl.rc.ch[4] = 0;
    rc_ctrl.rc.s[0] = RC_SW_DOWN;
    rc_ctrl.rc.s[1] = RC_SW_DOWN;
    rc_ctrl.mouse.x = 0;
    rc_ctrl.mouse.y = 0;
    rc_ctrl.mouse.z = 0;
    rc_ctrl.mouse.press_l = 0;
    rc_ctrl.mouse.press_r = 0;
    rc_ctrl.key.v = 0;
    return 1;
}


//取正函数
int16_t RC_abs(int16_t value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef * artHandle)       //串口中断回调函数
{  
		rc_ctrl.rc.ch[0] = (sbus_rx_buf[0] | (sbus_rx_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl.rc.ch[1] = ((sbus_rx_buf[1] >> 3) | (sbus_rx_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl.rc.ch[2] = ((sbus_rx_buf[2] >> 6) | (sbus_rx_buf[3] << 2) |          //!< Channel 2
                         (sbus_rx_buf[4] << 10)) &0x07ff;
    rc_ctrl.rc.ch[3] = ((sbus_rx_buf[4] >> 1) | (sbus_rx_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl.rc.s[0] = ((sbus_rx_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl.rc.s[1] = ((sbus_rx_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    rc_ctrl.mouse.x = sbus_rx_buf[6] | (sbus_rx_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl.mouse.y = sbus_rx_buf[8] | (sbus_rx_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl.mouse.z = sbus_rx_buf[10] | (sbus_rx_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl.mouse.press_l = sbus_rx_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl.mouse.press_r = sbus_rx_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl.key.v = sbus_rx_buf[14] | (sbus_rx_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl.rc.ch[4] = (sbus_rx_buf[16] | (sbus_rx_buf[17] << 8))& 0x07ff;                 //滚轮

    rc_ctrl.rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[4] -= RC_CH_VALUE_OFFSET;
}

