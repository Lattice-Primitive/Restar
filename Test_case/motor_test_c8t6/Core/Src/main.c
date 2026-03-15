/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CAN_receive.h"//通信
#include "bsp_can.h"//硬件初始化，屏蔽
#include "enginer_task.h"//电机参数
#include "pid.h"//闭环控制算法
#include "remote_control.h"//遥控
//#include "openmv.h"//阵营
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
fp32 map(fp32 x, fp32 in_min, fp32 in_max, fp32 out_min, fp32 out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
} 

//旋转速度转换比例

/**
  * @brief          四个麦轮速度是通过三个参数计算出来的
  * @param[in]      vx_set: 纵向速度
  * @param[in]      vy_set: 横向速度
  * @param[in]      wz_set: 旋转速度
  * @param[out]     wheel_speed: 四个麦轮速度（m/s，需换算）
  * @retval         none
  */

void chassis_vector_to_mecanum_wheel_speed(const float vx_set, const float vy_set, const float wz_set, float wheel_speed[4])
{
    wheel_speed[0] =( -vx_set - vy_set )*0.7f+  wz_set*0.5f;
    wheel_speed[1] = (vx_set - vy_set )*0.7f+   wz_set*0.5f;
    wheel_speed[2] = (vx_set + vy_set)*0.7f +   wz_set*0.5f;
    wheel_speed[3] = (-vx_set + vy_set)*0.7f +  wz_set*0.5f;
}



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
fp32 M2006_SPEED_pid[3]={M2006_MOTOR_SPEED_PID_KP,M2006_MOTOR_SPEED_PID_KI,M2006_MOTOR_SPEED_PID_KD};    //M2006电机速度环pid
fp32 M2006_POSITION_pid[3]={M2006_MOTOR_POSITION_PID_KP,M2006_MOTOR_POSITION_PID_KI,M2006_MOTOR_POSITION_PID_KD};    //M2006电机位置环pid

fp32 M3508_SPEED_pid[3]={M3508_MOTOR_SPEED_PID_KP,M3508_MOTOR_SPEED_PID_KI,M3508_MOTOR_SPEED_PID_KD};    //M3508电机速度环pid
fp32 M3508_POSITION_pid[3]={M3508_MOTOR_POSITION_PID_KP,M3508_MOTOR_POSITION_PID_KI,M3508_MOTOR_POSITION_PID_KD};    //M3508电机位置环pid

fp32 M6020_SPEED_pid[3]={M6020_MOTOR_SPEED_PID_KP,M6020_MOTOR_SPEED_PID_KI,M6020_MOTOR_SPEED_PID_KD};    //M6020电机速度环pid
fp32 M6020_POSITION_pid[3]={M6020_MOTOR_POSITION_PID_KP,M6020_MOTOR_POSITION_PID_KI,M6020_MOTOR_POSITION_PID_KD};    //M6020电机位置环pid

extern enginer_control_t enginer_move[8];    //电机控制参数结构体变量 
extern motor_measure_t motor_can1_data[8];   //can1接收的电机数据 
extern uint8_t sbus_rx_buf[SBUS_RX_BUF_NUM];   //遥控器接收缓存数据 
extern RC_ctrl_t rc_ctrl;   //遥控器储存结构体变量
extern  uint8_t  can_send_data[8];


uint16_t angle_var = 500,angle_flag = 0;   //舵机变量

float Motor_speed[4];

uint16_t motor_flag = 0;

uint8_t flag_sta = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	can_filter_init();  //在主函数初始化调用一遍，将CAN屏蔽功能去掉
	HAL_UART_Receive_DMA(&huart3,sbus_rx_buf,18);    //开启串口DMA接收，只需要开一遍就可以轮询接收 
	
	//初始化0号为2006电机
	enginer_init(&enginer_move[0], M2006_SPEED_pid, M2006_POSITION_pid, M2006_MOTOR_SPEED_PID_MAX_OUT, M2006_MOTOR_SPEED_PID_MAX_IOUT, \
								M2006_MOTOR_POSITION_PID_MAX_OUT, M2006_MOTOR_POSITION_PID_MAX_IOUT,0);
//	enginer_init(&enginer_move[0], M3508_SPEED_pid, M3508_POSITION_pid, M3508_MOTOR_SPEED_PID_MAX_OUT, M3508_MOTOR_SPEED_PID_MAX_IOUT, \
//M3508_MOTOR_POSITION_PID_MAX_OUT, M3508_MOTOR_POSITION_PID_MAX_IOUT,0); 
	//初始化1号为2006电机
	enginer_init(&enginer_move[1], M2006_SPEED_pid, M2006_POSITION_pid, M2006_MOTOR_SPEED_PID_MAX_OUT, M2006_MOTOR_SPEED_PID_MAX_IOUT, \
								M2006_MOTOR_POSITION_PID_MAX_OUT, M2006_MOTOR_POSITION_PID_MAX_IOUT,1);
//	enginer_init(&enginer_move[1], M3508_SPEED_pid, M3508_POSITION_pid, M3508_MOTOR_SPEED_PID_MAX_OUT, M3508_MOTOR_SPEED_PID_MAX_IOUT, \
//M3508_MOTOR_POSITION_PID_MAX_OUT, M3508_MOTOR_POSITION_PID_MAX_IOUT,1); 					
	//初始化2号为2006电机
	enginer_init(&enginer_move[2], M2006_SPEED_pid, M2006_POSITION_pid, M2006_MOTOR_SPEED_PID_MAX_OUT, M2006_MOTOR_SPEED_PID_MAX_IOUT, \
								M2006_MOTOR_POSITION_PID_MAX_OUT, M2006_MOTOR_POSITION_PID_MAX_IOUT,2);
//	enginer_init(&enginer_move[2], M3508_SPEED_pid, M3508_POSITION_pid, M3508_MOTOR_SPEED_PID_MAX_OUT, M3508_MOTOR_SPEED_PID_MAX_IOUT, \
//M3508_MOTOR_POSITION_PID_MAX_OUT, M3508_MOTOR_POSITION_PID_MAX_IOUT,2); 					
	//初始化3号为2006电机
	enginer_init(&enginer_move[3], M2006_SPEED_pid, M2006_POSITION_pid, M2006_MOTOR_SPEED_PID_MAX_OUT, M2006_MOTOR_SPEED_PID_MAX_IOUT, \
								M2006_MOTOR_POSITION_PID_MAX_OUT, M2006_MOTOR_POSITION_PID_MAX_IOUT,3);
//	enginer_init(&enginer_move[3], M3508_SPEED_pid, M3508_POSITION_pid, M3508_MOTOR_SPEED_PID_MAX_OUT, M3508_MOTOR_SPEED_PID_MAX_IOUT, \
//M3508_MOTOR_POSITION_PID_MAX_OUT, M3508_MOTOR_POSITION_PID_MAX_IOUT,3); 	
        enginer_init(&enginer_move[4], M2006_SPEED_pid, M2006_POSITION_pid, M2006_MOTOR_SPEED_PID_MAX_OUT, M2006_MOTOR_SPEED_PID_MAX_IOUT, \
								M2006_MOTOR_POSITION_PID_MAX_OUT, M2006_MOTOR_POSITION_PID_MAX_IOUT,4);
	//给初值
	enginer_move[0].motor.target_speed = 0.0f;
	enginer_move[1].motor.target_speed = 0.0f;
	enginer_move[2].motor.target_speed = 0.0f;
	enginer_move[3].motor.target_speed = 0.0f;
	enginer_move[4].motor.target_speed = 0.0f;
	//开启定时器1中断
	HAL_TIM_Base_Start_IT(&htim1);
	
	//开启PWM各通道
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	
	HAL_TIM_Base_Start(&htim4);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	
	HAL_Delay(500);
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
		if(K1){
			//蜂鸣器呼叫
			HAL_GPIO_WritePin(BUZZER_GPIO_Port,BUZZER_Pin,1);
			//点亮LED灯
			HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,0);
			HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,0);
			HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,0);
		}
		if(K2){
			//蜂鸣器关闭
			HAL_GPIO_WritePin(BUZZER_GPIO_Port,BUZZER_Pin,0);
            //红方
          //  openmv_red(void);
		}
		if(K3){
			//关闭LED灯
			HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,1);
			HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,1);
			HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,1);
            //蓝方
            
		}
		
		
		if(rc_ctrl.rc.s[0] == 2){   //遥控器右拨杆往下 输出速度为0
			enginer_move[0].motor.target_speed = 0.0f;
			enginer_move[1].motor.target_speed = 0.0f;
			enginer_move[2].motor.target_speed = 0.0f;
			enginer_move[3].motor.target_speed = 0.0f;
		}
		else{
			//舵机驱动  500~2500  对应0~180°
			if(angle_flag == 0 && angle_var < 2500){
				angle_var += 10;
			}
			else{
				angle_flag = 1;
			}
			
			if(angle_flag == 1 && angle_var > 500){
				angle_var -= 10;
			}
			else{
				angle_flag = 0;
			}
			//输出PWM控制舵机
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, angle_var);
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, angle_var);
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, angle_var);
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, angle_var);
			__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, angle_var);
			__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, angle_var);

			
//			//将遥控器通道值映射到2006电机的速度控制 
//			enginer_move[0].motor.target_speed = map(rc_ctrl.rc.ch[0], -660, 660, -10000, 10000);
//			enginer_move[1].motor.target_speed = map(rc_ctrl.rc.ch[1], -660, 660, -10000, 10000);
//			enginer_move[2].motor.target_speed = map(rc_ctrl.rc.ch[2], -660, 660, -10000, 10000);
//			enginer_move[3].motor.target_speed = map(rc_ctrl.rc.ch[3], -660, 660, -10000, 10000);
            
            //斜角平移
//            if(flag_sta == 0){
//               chassis_vector_to_mecanum_wheel_speed(0.0,0.0,-2000.0,Motor_speed);
//            }
//            if(flag_sta == 1){
//                chassis_vector_to_mecanum_wheel_speed(500.0,3000.0,0.0,Motor_speed);
//            }
//            if (flag_sta == 2){
//                chassis_vector_to_mecanum_wheel_speed(0.0,0.0,0.0,Motor_speed);
//            }

//              if(flag_sta == 0){
//               chassis_vector_to_mecanum_wheel_speed(3000.0,0.0,0.0,Motor_speed);
//              }
//             if(flag_sta == 1){
//                 chassis_vector_to_mecanum_wheel_speed(0.0,3000.0,0.0,Motor_speed);
//             }
//             if (flag_sta == 2){
//                 chassis_vector_to_mecanum_wheel_speed(-3000.0,0.0,0.0,Motor_speed);
//             }
//             if(flag_sta == 3){
//                 chassis_vector_to_mecanum_wheel_speed(0.0,-3000.0,0.0,Motor_speed);
//             }
//             if(flag_sta == 4){
//                 chassis_vector_to_mecanum_wheel_speed(0.0,0.0,0.0,Motor_speed);
//             
//             }
            //遥控
            enginer_move[0].motor.target_speed = 1000.0f;
//            chassis_vector_to_mecanum_wheel_speed(rc_ctrl.rc.ch[1],-rc_ctrl.rc.ch[0],rc_ctrl.rc.ch[2],Motor_speed);

//			enginer_move[0].motor.target_speed = Motor_speed[0]*8.0f;
//			enginer_move[1].motor.target_speed = Motor_speed[1]*8.0f;
//			enginer_move[2].motor.target_speed = Motor_speed[2]*8.0f;
//			enginer_move[3].motor.target_speed = Motor_speed[3]*8.0f;
	
		}
//        enginer_move[4].motor.target_speed = 2700.0f;
		HAL_Delay(10);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//定时溢出中断回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	//定时器1专门用于发数据给电机
	if(htim->Instance == TIM1){    //10ms进入1次
//        if(motor_flag  <900){
//        motor_flag++;
//        }
//        if(motor_flag >= 200){ 
//            
//            flag_sta = 1; 
//        }
//        if(motor_flag >=400){           
//             flag_sta = 2; 
//        }
//        if(motor_flag >=600){           
//             flag_sta = 3; 
//        }
//        if(motor_flag >=800){           
//             flag_sta = 4; 
//        }
		//PID控制闭环  （电机控制参数结构体变量，控制模式:0速度1角度，电机角度比） 
		PIDrealize(&enginer_move[0],0,MOTOR_ECD_TO_ANGLE_2006);
		PIDrealize(&enginer_move[1],0,MOTOR_ECD_TO_ANGLE_2006);
		PIDrealize(&enginer_move[2],0,MOTOR_ECD_TO_ANGLE_2006);
		PIDrealize(&enginer_move[3],0,MOTOR_ECD_TO_ANGLE_2006);
//        PIDrealize(&enginer_move[0],0,MOTOR_ECD_TO_ANGLE_3508);
//		PIDrealize(&enginer_move[1],0,MOTOR_ECD_TO_ANGLE_3508);
//		PIDrealize(&enginer_move[2],0,MOTOR_ECD_TO_ANGLE_3508);
//        PIDrealize(&enginer_move[3],0,MOTOR_ECD_TO_ANGLE_3508);
//		PIDrealize(&enginer_move[4],0,MOTOR_ECD_TO_ANGLE_2006);
		//将解算好的电流发送出去给电机
		CAN1_cmd_0_3(enginer_move[0].motor.give_current,enginer_move[1].motor.give_current,enginer_move[2].motor.give_current,enginer_move[3].motor.give_current);
		CAN1_cmd_4_7(enginer_move[4].motor.give_current,enginer_move[5].motor.give_current,enginer_move[6].motor.give_current,enginer_move[7].motor.give_current);
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
