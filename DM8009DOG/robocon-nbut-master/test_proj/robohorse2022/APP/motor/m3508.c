/**
 * 电机驱动部分程序
 * CAN1上连接了ID1~4的电机
 * CAN2上连接了ID5~8的电机
 * */
#include "m3508.h"
#include "main.h"
#include "pid.h"
#include "gait_param.h"
#include "can.h"
#include "usart.h"

PID_t PID_Position[8];  //位置PID
PID_t PID_Speed[8];     //速度PID
PID_t PID_IMU_Climbing; //爬坡用PID
PID_t PID_IMU_Line;     //直线修正pid

M3508_Measure_t M3508[8]; //电机测量值

// 陀螺仪数据结构
typedef struct {
    float pitch;
    float roll;
    float yaw;
    uint8_t is_data_valid;
} IMU_Data_t;

IMU_Data_t imu_data = {0};

// 陀螺仪数据解析函数
void IMU_Data_Parse(uint8_t *data)
{
    // 假设数据格式为: pitch(4字节) + roll(4字节) + yaw(4字节)
    float *pitch = (float*)&data[0];
    float *roll = (float*)&data[4];
    float *yaw = (float*)&data[8];
    
    imu_data.pitch = *pitch;
    imu_data.roll = *roll;
    imu_data.yaw = *yaw;
    imu_data.is_data_valid = 1;
}

// 串口接收回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    static uint8_t rx_buffer[12];  // 12字节数据缓冲区
    
    if(huart->Instance == USART1)  // 假设使用USART1
    {
        IMU_Data_Parse(rx_buffer);
        // 重新启动接收
        HAL_UART_Receive_IT(&huart1, rx_buffer, 12);
    }
}

// 获取陀螺仪数据
void Get_IMU_Data(float *pitch, float *roll, float *yaw)
{
    if(imu_data.is_data_valid)
    {
        *pitch = imu_data.pitch;
        *roll = imu_data.roll;
        *yaw = imu_data.yaw;
    }
}

// 配置并启动CAN，并设置PID数据
void M3508_Init()
{
    //初始化CAN过滤器
    CAN_FilterTypeDef can_filter =
        {
            0x0000,
            0x0000,
            0x0000,
            0x0000,
            CAN_FILTER_FIFO0,
            0,
            CAN_FILTERMODE_IDMASK,
            CAN_FILTERSCALE_32BIT,
            CAN_FILTER_ENABLE,
            14};
    HAL_CAN_ConfigFilter(&hcan1, &can_filter); //设置CAN1过滤器
    can_filter.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter); //设置CAN2过滤器
    
    // 修改电机ID映射关系
    // CAN1: ID1-4 -> 右前腿(0,1), 右后腿(2,3)
    // CAN2: ID5-8 -> 左前腿(4,5), 左后腿(6,7)
    
    PID_Set_t pid_set =
        {
            .pid_mode = POSITION_PID,
            .MaxOutput = 6720.0f,
            .IntegralLimit = 2000.0f,
            .p = 8.0f,
            .i = 0.0008f,
            .d = 0.0f,
        };
    for (uint8_t i = 0; i < 8; i++)
    {
        PID_Init(&PID_Position[i], &pid_set); //设置位置式PID
    }
    pid_set.MaxOutput = 12000.0f;
    pid_set.IntegralLimit = 2000.0f;
    pid_set.p = 15.5f;
    pid_set.i = 0.0001f;
    for (uint8_t i = 0; i < 8; i++)
    {
        PID_Init(&PID_Speed[i], &pid_set); //设置速度式PID
    }

    pid_set.MaxOutput = 10000.0f;
    pid_set.IntegralLimit = 0.0f;
    pid_set.p = 3.5f;
    pid_set.i = 0.0f;
    pid_set.d = 0.0f;
    PID_Init(&PID_IMU_Climbing, &pid_set); //设置爬坡PID

    pid_set.MaxOutput = 16.0f;
    pid_set.IntegralLimit = 0.0f;
    pid_set.p = 1.5f;
    pid_set.i = 0.0f;
    pid_set.d = 0.01f;
    PID_Init(&PID_IMU_Line, &pid_set);

    for (uint8_t i = 0; i < 4; i++)
    {
        Stanford_Type_Lite_Init(&Gait_Data[i], 10.0f, 20.0f); //设置腿部长度参数
    }

    HAL_CAN_Start(&hcan1);                                             // CAN1开始运行
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); // CAN1开始接受数据
    HAL_CAN_Start(&hcan2);                                             // CAN2开始运行
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING); // CAN2开始接受数据

    // 初始化陀螺仪串口
    uint8_t rx_buffer[12];
    HAL_UART_Receive_IT(&huart1, rx_buffer, 12);
}

/*电机输出电流控制*/
/**
 * @brief 电机输出电流控制
 * @param currents (M3508_OutputCurrent_t) 各电机电流值结构体
 */
void M3508_SetCurrent(M3508_OutputCurrent_t *currents)
{
    CAN_TxHeaderTypeDef can_header = {
        0x200,
        0x0,
        CAN_ID_STD,
        CAN_RTR_DATA,
        8,
        DISABLE};
    //装填电机输出值
    uint8_t rx_data[8];
    rx_data[0] = (uint8_t)((currents->ID1 & 0xff00) >> 8);
    rx_data[1] = (uint8_t)(currents->ID1 & 0x00ff);
    rx_data[2] = (uint8_t)((currents->ID2 & 0xff00) >> 8);
    rx_data[3] = (uint8_t)(currents->ID2 & 0x00ff);
    rx_data[4] = (uint8_t)((currents->ID3 & 0xff00) >> 8);
    rx_data[5] = (uint8_t)(currents->ID3 & 0x00ff);
    rx_data[6] = (uint8_t)((currents->ID4 & 0xff00) >> 8);
    rx_data[7] = (uint8_t)(currents->ID4 & 0x00ff);
    HAL_CAN_AddTxMessage(&hcan1, &can_header, rx_data, NULL);

    can_header.StdId = 0x1ff;
    rx_data[0] = (uint8_t)((currents->ID5 & 0xff00) >> 8);
    rx_data[1] = (uint8_t)(currents->ID5 & 0x00ff);
    rx_data[2] = (uint8_t)((currents->ID6 & 0xff00) >> 8);
    rx_data[3] = (uint8_t)(currents->ID6 & 0x00ff);
    rx_data[4] = (uint8_t)((currents->ID7 & 0xff00) >> 8);
    rx_data[5] = (uint8_t)(currents->ID7 & 0x00ff);
    rx_data[6] = (uint8_t)((currents->ID8 & 0xff00) >> 8);
    rx_data[7] = (uint8_t)(currents->ID8 & 0x00ff);
    HAL_CAN_AddTxMessage(&hcan2, &can_header, rx_data, NULL);
}

void get_moto_measure(M3508_Measure_t *ptr)
{
    if (ptr->Rotor_Angle - ptr->Last_Angle > 4096)
        ptr->Round_Cnt--;
    else if (ptr->Rotor_Angle - ptr->Last_Angle < -4096)
        ptr->Round_Cnt++;
    ptr->Total_Angle = ptr->Round_Cnt * 8192 + ptr->Rotor_Angle - ptr->Offset_Angle;
    ptr->Last_Angle = ptr->Rotor_Angle;
}

/*this function should be called after system+can init */
void get_moto_offset(M3508_Measure_t *ptr)
{
    ptr->Offset_Angle = ptr->Rotor_Angle;
    ptr->Last_Angle = ptr->Rotor_Angle;
}

/*接收数据*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef can_header;
    uint8_t rx_data[8];
    if ((hcan == &hcan1) || (hcan == &hcan2))
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can_header, rx_data);
        if (0x201 <= can_header.StdId && can_header.StdId <= 0x208)
        {
            uint16_t rx_motor = can_header.StdId - 0x201;
            M3508[rx_motor].Rotor_Angle = (int16_t)(((uint16_t)rx_data[0]) << 8 | rx_data[1]);
            M3508[rx_motor].Speed_RPM = (int16_t)(((uint16_t)rx_data[2]) << 8 | rx_data[3]);
            M3508[rx_motor].Real_Current = (int16_t)(((uint16_t)rx_data[4]) << 8 | rx_data[5]);
            M3508[rx_motor].Temp = rx_data[6];

            if (M3508[rx_motor].Msg_Cnt <= 50)
            {
                get_moto_offset(&M3508[rx_motor]);
                M3508[rx_motor].Msg_Cnt++;
            }
            else
            {
                get_moto_measure(&M3508[rx_motor]);
            }
        }
    }
}
