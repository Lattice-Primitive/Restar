/**
 * 电机驱动部分程序
 * CAN1上连接了ID1~4的电机
 * CAN2上连接了ID5~8的电机
 * */
#include "dm8009.h"
#include "main.h"
#include "pid.h"
#include "gait_param.h"
#include "can.h"
#include "dm8009_drv.h"
PID_t PID_Position[8];  //位置PID
//PID_t PID_Speed[8];     //速度PID
PID_t PID_IMU_Climbing; //爬坡用PID
PID_t PID_IMU_Line;     //直线修正pid

extern DM_J8009_DATA_t dm_j8009_data[8];		//达妙电机反馈数据

// 配置并启动CAN，并设置PID数据
void DM8009_Init()
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
            .p = 60.0f,//(80)
            .i = 0.0008f,
            .d = 2.0f,
        };
    for (uint8_t i = 0; i < 8; i++)
    {
        PID_Init(&PID_Position[i], &pid_set); //设置位置式PID
    }
//    pid_set.MaxOutput = 12000.0f;
//    pid_set.IntegralLimit = 2000.0f;
//    pid_set.p = 15.5f;
//    pid_set.i = 0.0001f;
//    for (uint8_t i = 0; i < 8; i++)
//    {
//        PID_Init(&PID_Speed[i], &pid_set); //设置速度式PID
//    }

    pid_set.p = 3.5f;
    pid_set.i = 0.0f;
    pid_set.d = 0.0f;
    PID_Init(&PID_IMU_Climbing, &pid_set); //设置爬坡PID

    pid_set.MaxOutput = 16.0f;
    pid_set.IntegralLimit = 0.0f;
    pid_set.p = 1.5f;
    pid_set.i = 0.0f;
    pid_set.d = 0.01f;
    PID_Init(&PID_IMU_Line, &pid_set);//直线

    for (uint8_t i = 0; i < 4; i++)
    {
        Stanford_Type_Lite_Init(&Gait_Data[i], 12.0f, 20.0f); //设置腿部长度参数
    }

    HAL_CAN_Start(&hcan1);                                             // CAN1开始运行
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); // CAN1开始接受数据
    HAL_CAN_Start(&hcan2);                                             // CAN2开始运行
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING); // CAN2开始接受数据
}


/*接收数据*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef can_header;
    uint8_t rx_data[8];
	
	if(hcan->Instance == CAN1 || hcan->Instance == CAN2)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can_header, rx_data);
	  if (can_header.StdId <= 0x07)			
		{	
			//达妙电机反馈数据
			FEEDBACK_DATA_DM_J8009(can_header.StdId , rx_data);
		}
	}
//	
//    if ((hcan == &hcan1) || (hcan == &hcan2))
//    {
//        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can_header, rx_data);
//        if (0x201 <= can_header.StdId && can_header.StdId <= 0x208)
//        {
//            uint16_t rx_motor = can_header.StdId - 0x201;
//            M3508[rx_motor].Rotor_Angle = (int16_t)(((uint16_t)rx_data[0]) << 8 | rx_data[1]);
//            M3508[rx_motor].Speed_RPM = (int16_t)(((uint16_t)rx_data[2]) << 8 | rx_data[3]);
//            M3508[rx_motor].Real_Current = (int16_t)(((uint16_t)rx_data[4]) << 8 | rx_data[5]);
//            M3508[rx_motor].Temp = rx_data[6];

//            if (M3508[rx_motor].Msg_Cnt <= 50)
//            {
//                get_moto_offset(&M3508[rx_motor]);
//                M3508[rx_motor].Msg_Cnt++;
//            }
//            else
//            {
//                get_moto_measure(&M3508[rx_motor]);
//            }
//        }
//    }
}
