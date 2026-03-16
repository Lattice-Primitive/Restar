#include "remoctrl_task.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "usart.h"
#include "pid.h"
#include "gait_param.h"
#include "app_btdebug.h"
#include "serial.h"
#include "posture_task.h"
#include "hwt906.h"
#include "dm_j8009_ctrl.h"



#define TX_DATAF32_SIZE (8) //机器马发送数据数量(float32)
#define RX_DATA32_SIZE (1)  //机器马接收数据数量(int32)

#define TX_DATA8_SIZE (TX_DATAF32_SIZE * 4) //机器马发送数据数量(uint8)
#define RX_DATA8_SIZE (RX_DATA32_SIZE * 4)  //机器马接受数据数量(uint8)

uint32_t get_num = 0;      //接受的数据
uint8_t flag_imu_line = 0; // imu直线修正标志

extern PID_t PID_Position[8];
extern PID_t PID_Speed[8];


extern float Target_Angle;
extern AHRS_Angle_t Now_Angle;

//向蓝牙发送的数据
union
{
    uint8_t data8[TX_DATA8_SIZE];
    float dataf32[TX_DATAF32_SIZE];
} bt_tx_data;

//向蓝牙读取的数据
union
{
    uint8_t data8[RX_DATA8_SIZE];
    uint32_t data32[RX_DATA32_SIZE];
} bt_rx_data;

void Ctrl_byRemoctrl(Cycloid_Generator_t *pCycGenerato, RC_ctrl_t *pRCData);
void Change_RmVAL(Cycloid_Generator_t *pCycGenerato, RC_ctrl_t *pRCData, Robohorse_State state);

void _UART_Transmit_Bytes(uint8_t *pData, uint32_t Size)
{
    HAL_UART_Transmit(&huart6, pData, Size, 100);
}

void _UART_Receive_Bytes(uint8_t *pData, uint32_t Size)
{
    HAL_UART_Receive(&huart6, pData, Size, 100);
}

BTDebug_Operate_t btdbg_opt =
    {
        .ReceiveBytes = _UART_Receive_Bytes,
        .TransmitBytes = _UART_Transmit_Bytes,
};

/*遥控器接收任务*/
void RemoCtrl_Task(void *argument)
{
    while (1)
    {
        /* 进入临界段保护 */
        taskENTER_CRITICAL();

        RC_Transform(rc_data, &my_rc_ctrl);

        //填充数据
        bt_tx_data.dataf32[0] = Gait_Data[0].position[0];
        bt_tx_data.dataf32[1] = Gait_Data[0].position[1];
        bt_tx_data.dataf32[2] = Gait_Data[0].angle[0];
        bt_tx_data.dataf32[3] = Gait_Data[0].angle[1];
        bt_tx_data.dataf32[4] = PID_Position[0].set[0];
        bt_tx_data.dataf32[5] = PID_Speed[0].pos_out;
        bt_tx_data.dataf32[6] = PID_Position[1].set[0];
        bt_tx_data.dataf32[7] = PID_Speed[1].pos_out;

        /* 退出临界段保护 */
        taskEXIT_CRITICAL();

        //发送数据
        BTDebug_TransmitPackage(&btdbg_opt, bt_tx_data.data8, TX_DATA8_SIZE);
        // BTDebug_ReceivePackage(&btdbg_opt,bt_rx_data.data8,RX_DATA8_SIZE);
        // get_num=bt_rx_data.data32[0];
        osDelay(10);
    }
}

/**
 * @brief 根据遥控器的值修改步态参数，可以调整开关状态下的步态模式
 * @param pCycGenerato 摆线生成器地址
 * @param pRCData 遥控器数据地址
 * @return void
 */
void Ctrl_byRemoctrl(Cycloid_Generator_t *pCycGenerato, RC_ctrl_t *pRCData)
{
    static RC_Key now_sw_state = RC_NULL;

    /* 紧急停止 */
    if (rc_sw_state == RC_11)
    {
        now_sw_state = RC_11;
        Change_NowState(STOP);
        dm_j8009_all_disable();
        flag_imu_line = 0;
        return;
    }
    else if (rc_sw_state == RC_12 || rc_sw_state == RC_13)
    {
        now_sw_state = rc_sw_state;
        dm_j8009_all_enable();
        Change_NowState(STOP);
        return;
    }
    else
    {
        dm_j8009_all_enable();
    }


    /*遥控器无操作时切换至STOP状态 */
    if (pRCData->rc.ch[0] == 1024 && pRCData->rc.ch[1] == 1024 &&
        pRCData->rc.ch[2] == 1024 && pRCData->rc.ch[3] == 1024)
    {
        osDelay(5);
        if (pRCData->rc.ch[0] == 1024 && pRCData->rc.ch[1] == 1024 &&
            pRCData->rc.ch[2] == 1024 && pRCData->rc.ch[3] == 1024)
            Change_NowState(STOP);

        return;
    }
    /* 防止切换开关时出现问题 */
    if (now_sw_state != rc_sw_state)
    {
        RC_Key last_sw_state = rc_sw_state;
        osDelay(500);
        if (now_sw_state != rc_sw_state && rc_sw_state == last_sw_state)
        {
            now_sw_state = rc_sw_state;
        }
    }
    if (pRCData->rc.ch[4] > 1600)
    {
        Target_Angle = Now_Angle.Yaw;
        flag_imu_line = 1;
    }
    else if (pRCData->rc.ch[4] < 400)
    {
        flag_imu_line = 0;
    }
    taskENTER_CRITICAL();



    switch (now_sw_state)
    {
    /* WALK完全遥控模式 */
    case RC_31://中上
         Change_RmVAL(pCycGenerato, pRCData, WALK);
        break;
    case RC_33://中中
 //       Change_RmVAL(pCycGenerato, pRCData, BRIDGE);
         Change_RmVAL(pCycGenerato, pRCData, TROT);
        break;
    case RC_32://中下      
   //        flag_imu_line = 0;
        if (pRCData->rc.ch[1] > 1600 && pRCData->rc.ch[3] > 1600)
        {
            Change_NowState(JUMP_UPHILL);
        }
        else if (pRCData->rc.ch[1] < 400 && pRCData->rc.ch[3] < 400)
        {
            Change_NowState(JUMP_GROUND);
        }
        break;

    case RC_21://下上
        Change_RmVAL(pCycGenerato, pRCData, CLIMBING);
        break;
    /* TROT完全遥控模式 */
    case RC_23://下中
        Change_RmVAL(pCycGenerato, pRCData, BRIDGE);
        break;
    /* 跳跃控制模式*/
    case RC_22://下下
        // //        flag_imu_line = 0;
        // if (pRCData->rc.ch[1] > 1600 && pRCData->rc.ch[3] > 1600)
        // {
        //     Change_NowState(JUMP_UPHILL);
        // }
        // else if (pRCData->rc.ch[1] < 400 && pRCData->rc.ch[3] < 400)
        // {
        //     Change_NowState(JUMP_GROUND);
        // }
        
        break;
    /* 爬坡模式 */
    
    default:
        break;
    }
    taskEXIT_CRITICAL();
}

/**
 * @brief 遥控器数值转化步幅控制
 * @param pCycGenerato 摆线生成器地址
 * @param pRCData 遥控器数据地址
 * @param state  步态
 *
 */
void Change_RmVAL(Cycloid_Generator_t *pCycGenerato, RC_ctrl_t *pRCData, Robohorse_State state)
{
    float
        CH0_Offset = ((pRCData->rc.ch[0]) - 1024) / 660.0f,
        CH1_Offset = ((pRCData->rc.ch[1]) - 1024) / 660.0f,
        CH2_Offset = ((pRCData->rc.ch[2]) - 1024) / 660.0f,
        CH3_Offset = ((pRCData->rc.ch[3]) - 1024) / 660.0f;

    if (state == TROT || state == WALK || state == CLIMBING || state == BRIDGE)
    {
        if (pRCData->rc.ch[0] == 1024 && pRCData->rc.ch[1] == 1024)
        {
            pCycGenerato[0].param.step_length = RC_CPLTCTRL_STEPLENTH_MAX * (CH3_Offset + CH2_Offset);
            pCycGenerato[1].param.step_length = RC_CPLTCTRL_STEPLENTH_MAX * (CH3_Offset + CH2_Offset);
            pCycGenerato[2].param.step_length = RC_CPLTCTRL_STEPLENTH_MAX * (CH3_Offset - CH2_Offset);
            pCycGenerato[3].param.step_length = RC_CPLTCTRL_STEPLENTH_MAX * (CH3_Offset - CH2_Offset);
            LIMIT_PARAM(pCycGenerato[0].param.step_length, RC_CPLTCTRL_STEPLENTH_MAX);
            LIMIT_PARAM(pCycGenerato[1].param.step_length, RC_CPLTCTRL_STEPLENTH_MAX);
            LIMIT_PARAM(pCycGenerato[2].param.step_length, RC_CPLTCTRL_STEPLENTH_MAX);
            LIMIT_PARAM(pCycGenerato[3].param.step_length, RC_CPLTCTRL_STEPLENTH_MAX);
        }
        else
        {
            pCycGenerato[0].param.freq = 0.5f;
            pCycGenerato[1].param.freq = 0.5f;
            pCycGenerato[2].param.freq = 0.5f;
            pCycGenerato[3].param.freq = 0.5f;

            pCycGenerato[0].param.step_length = RC_CPLTCTRL_FINETUNE_STEPLENTH_MAX * (CH1_Offset + CH0_Offset);
            pCycGenerato[1].param.step_length = RC_CPLTCTRL_FINETUNE_STEPLENTH_MAX * (CH1_Offset + CH0_Offset);
            pCycGenerato[2].param.step_length = RC_CPLTCTRL_FINETUNE_STEPLENTH_MAX * (CH1_Offset - CH0_Offset);
            pCycGenerato[3].param.step_length = RC_CPLTCTRL_FINETUNE_STEPLENTH_MAX * (CH1_Offset - CH0_Offset);
            LIMIT_PARAM(pCycGenerato[0].param.step_length, RC_CPLTCTRL_FINETUNE_STEPLENTH_MAX);
            LIMIT_PARAM(pCycGenerato[1].param.step_length, RC_CPLTCTRL_FINETUNE_STEPLENTH_MAX);
            LIMIT_PARAM(pCycGenerato[2].param.step_length, RC_CPLTCTRL_FINETUNE_STEPLENTH_MAX);
            LIMIT_PARAM(pCycGenerato[3].param.step_length, RC_CPLTCTRL_FINETUNE_STEPLENTH_MAX);
        }
    }
    Change_NowState(state);
}
