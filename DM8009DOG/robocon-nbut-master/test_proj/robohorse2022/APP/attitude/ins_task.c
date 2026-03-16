#include "ins_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "CAN_receive.h"
#include "user_lib.h"
#include "pid.h"
#include "BMI088driver.h"
#include "MahonyAHRS.h"
#include "bsp_imu_pwm.h"
#include "math.h"
#include "tim.h"
#include "bsp_flash.h"


/**
  * @brief          陀螺仪校准任务
  * @param[in]      none
	* @retval         none
  */
void IMU_cali_task(void);

/**
  * @brief          根据陀螺仪的数据，加速度的数据，磁力计的数据进行四元数更新
  * @param[in]      需要更新的四元数数组
  * @param[in]      更新定时时间，固定定时调用，例如1000Hz，传入的数据为0.001f,
  * @param[in]      用于更新的陀螺仪数据,数组顺序(x,y,z) 单位 rad
  * @param[in]      用于初始化的加速度数据,数组顺序(x,y,z) 单位 m/s2 
  * @param[in]      用于初始化的磁力计数据,数组顺序(x,y,z) 单位 uT
  * @retval         none
  */
static void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3], fp32 mag[3]);

/**
  * @brief          根据四元数大小计算对应的欧拉角yaw，pitch，roll
  * @param[in]      四元数数组，不为NULL
  * @param[in]      返回的偏航角yaw 单位 rad
  * @param[in]      返回的俯仰角pitch  单位 rad
  * @param[in]      返回的横滚角roll 单位 rad
	* @retval         none
  */
static void get_angle(fp32 q[4], fp32 *yaw, fp32 *pitch, fp32 *roll);

/**
  * @brief          控制bmi088的温度
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
static void imu_temp_control(fp32 temp);


#if INCLUDE_uxTaskGetStackHighWaterMark
	uint32_t ins_high_water;
#endif

static TaskHandle_t INS_task_local_handler;


IMU_DATA imu_inside;
uint8_t cmd=0x01;         //初始化命令
fp32 mag[3];    				//磁力计原始数据
fp32 gyro[3], accel[3];    //BMI088原始数据
fp32 gyro_offset[3],accel_offset[3];  //零漂误差
uint8_t Gyro_OK,Accel_OK; //数据准备完成标志  1:可读取 0:未完成
fp32 control_temp = 40;       //要控制恒温的温度
uint8_t first_temperate;      //第一次恒温标志
pid_type_def imu_temp_pid;    //温度控制PID结构体
const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};     //PID参数
uint8_t temp_constant_time = 0;    	//初始温度加热到指定温度后继续加热时间

//加速度计低通滤波
fp32 accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
fp32 accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
fp32 accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

/**
  * @brief          imu任务, 初始化 bmi088, ist8310, 计算欧拉角
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void ins_task(void const *pvParameters)
{
	  //wait a time 
    //空闲一段时间
		osDelay(6);
		//开启定时器10通道1  恒温PWM控制通道
		HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);   
		//陀螺仪恒温控制初始化PID参数
		PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);
		//初始时间
		uint16_t init_time;   
		while(cmd){
			cmd = BMI088_init();     //BMI初始化  初始化成功为0x00
			vTaskDelay(1);  				//延迟1ms
			init_time < 2000 ? init_time ++ : init_time;
		}
		
		//读取gyro_offset零漂数据
		flash_read(ADDR_FLASH_SECTOR_11,(uint32_t *)gyro_offset,sizeof(gyro_offset));
		
		imu_inside.INS_quat[0] = 1;
		imu_inside.INS_quat[1] = 0;
		imu_inside.INS_quat[2] = 0;
		imu_inside.INS_quat[3] = 0;
		
		//获取当前任务的任务句柄，
    INS_task_local_handler = xTaskGetHandle(pcTaskGetName(NULL));
		
    while (1)
    {
			//获取陀螺仪数据
			if(Accel_OK == 1 && Gyro_OK == 1){    
				BMI088_read(gyro, accel, &imu_inside.temp);
				Accel_OK = 0;     
				Gyro_OK = 0;     
				imu_temp_control(imu_inside.temp);  //控制温度
			}
			//角速度减去零漂误差
			imu_inside.INS_gyro[0] = gyro[0] - gyro_offset[0];
			imu_inside.INS_gyro[1] = gyro[1] - gyro_offset[1];
			imu_inside.INS_gyro[2] = gyro[2] - gyro_offset[2];
			//加速度减去零漂误差
			imu_inside.INS_accel[0] = accel[0] - accel_offset[0];	
			imu_inside.INS_accel[1] = accel[1] - accel_offset[1];	
			imu_inside.INS_accel[2] = accel[2] - accel_offset[2];	
			//加速度计低通滤波
			accel_fliter_1[0] = accel_fliter_2[0];
			accel_fliter_2[0] = accel_fliter_3[0];
			accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + imu_inside.INS_accel[0] * fliter_num[2];
			accel_fliter_1[1] = accel_fliter_2[1];
			accel_fliter_2[1] = accel_fliter_3[1];
			accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + imu_inside.INS_accel[1] * fliter_num[2];
			accel_fliter_1[2] = accel_fliter_2[2];
			accel_fliter_2[2] = accel_fliter_3[2];
			accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + imu_inside.INS_accel[2] * fliter_num[2];
			//解算四元数
			AHRS_update(imu_inside.INS_quat,0.001f,imu_inside.INS_gyro,accel_fliter_3,mag);
			//四元数解算成角度  rad
			get_angle(imu_inside.INS_quat,&imu_inside.INS_angle[0],&imu_inside.INS_angle[1],&imu_inside.INS_angle[2]);
			
      //系统延时
      vTaskDelay(1);

#if INCLUDE_uxTaskGetStackHighWaterMark
      ins_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}



/**
  * @brief          陀螺仪校准任务
  * @param[in]      none
	* @retval         none
  */
void IMU_cali_task(void)
{
	vTaskSuspend(INS_task_local_handler);		//挂起IMU任务
	uint16_t cali_count = 0;              	//校准时间的计数值
	while(cali_count<20000){
		if(Accel_OK == 1 && Gyro_OK == 1){
			BMI088_read(gyro, accel, &imu_inside.temp);
			Accel_OK = 0;
			Gyro_OK = 0;     
			imu_temp_control(imu_inside.temp);   //恒温控制
			
			//角速度减去零漂误差
			imu_inside.INS_gyro[0] = gyro[0] - gyro_offset[0];
			imu_inside.INS_gyro[1] = gyro[1] - gyro_offset[1];
			imu_inside.INS_gyro[2] = gyro[2] - gyro_offset[2];
			//加速度减去零漂误差
			imu_inside.INS_accel[0] = accel[0] - accel_offset[0];	
			imu_inside.INS_accel[1] = accel[1] - accel_offset[1];	
			imu_inside.INS_accel[2] = accel[2] - accel_offset[2];	
			
			if(imu_inside.temp >= control_temp-0.5f && cali_count < 20000){       //达到恒温后才进行零漂的计算
				gyro_offset[0] += 0.0003f*imu_inside.INS_gyro[0];   								//计算零漂值
				gyro_offset[1] += 0.0003f*imu_inside.INS_gyro[1];
				gyro_offset[2] += 0.0003f*imu_inside.INS_gyro[2];
				
				//考虑到校准时可能不是水平放置以及重力加速度并非是9.8m/s2，对角速度采用低通滤波，无需校准
				//accel_offset[0] += 0.0003f*INS_ACCEL[0];
				//accel_offset[1] += 0.0003f*INS_ACCEL[1];
				//accel_offset[2] += 0.0003f*(INS_ACCEL[2]-9.8f);
				vTaskDelay(1);
				cali_count++;
				
			}
			else if(cali_count >= 20000){    //达到时间存储零漂
				//储存数据
//				Save_Data();
				
				
				
				//加速度低通滤波数据初始化
				accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = imu_inside.INS_accel[0];
				accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = imu_inside.INS_accel[1];
				accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = imu_inside.INS_accel[2];
				//四元数初始化
				imu_inside.INS_quat[0] = 1;
				imu_inside.INS_quat[1] = 0;
				imu_inside.INS_quat[2] = 0;
				imu_inside.INS_quat[3] = 0;
				return;
			}
		}
		
//		if(cali_count%500 == 0){        //红灯常亮、绿灯1Hz闪烁  每0.5s翻转绿灯电平达到效果
//			HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_SET);     	//红灯  亮
//			HAL_GPIO_TogglePin(LED_G_GPIO_Port,LED_G_Pin);   								//绿灯  翻转电平
//		}
	}
	vTaskResume(INS_task_local_handler);   //恢复IMU任务
}




/**
  * @brief          根据陀螺仪的数据，加速度的数据，磁力计的数据进行四元数更新
  * @param[in]      需要更新的四元数数组
  * @param[in]      更新定时时间，固定定时调用，例如1000Hz，传入的数据为0.001f,
  * @param[in]      用于更新的陀螺仪数据,数组顺序(x,y,z) 单位 rad
  * @param[in]      用于初始化的加速度数据,数组顺序(x,y,z) 单位 m/s2 
  * @param[in]      用于初始化的磁力计数据,数组顺序(x,y,z) 单位 uT
  * @retval         none
  */
void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3], fp32 mag[3])
{
    MahonyAHRSupdate(quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], mag[0], mag[1], mag[2]);
}



/**
  * @brief          根据四元数大小计算对应的欧拉角yaw，pitch，roll
  * @param[in]      四元数数组，不为NULL
  * @param[in]      返回的偏航角yaw 单位 rad
  * @param[in]      返回的俯仰角pitch  单位 rad
  * @param[in]      返回的横滚角roll 单位 rad
	* @retval         none
  */
void get_angle(fp32 q[4], fp32 *yaw, fp32 *pitch, fp32 *roll)
{
    *yaw = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f);
    *pitch = asinf(-2.0f*(q[1]*q[3]-q[0]*q[2]));
    *roll = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f);
}



/**
  * @brief          控制bmi088的温度
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
void imu_temp_control(fp32 temp)
{
    uint16_t tempPWM;
    if (first_temperate)
    {
        PID_calc(&imu_temp_pid, temp, control_temp);    //PID计算
        if (imu_temp_pid.out < 0.0f)   //限副，PWM没有负值
        {
            imu_temp_pid.out = 0.0f;
        }
        tempPWM = (uint16_t)imu_temp_pid.out;
        imu_pwm_set(tempPWM);
    }
    else
    {
        //在没有达到设置的温度，一直最大功率加热
        if (temp > control_temp)
        {
					temp_constant_time++;
					if (temp_constant_time > 200)
					{
							//达到设置温度，将积分项设置为一半最大功率，加速收敛
							first_temperate = 1;
							imu_temp_pid.Iout = MPU6500_TEMP_PWM_MAX / 2.0f;
					}
				}
        imu_pwm_set(MPU6500_TEMP_PWM_MAX - 1);
    }
}



/**
  * @brief          返回内置IMU数据指针
  * @param[in]      nonoe
  * @retval         none
  */
IMU_DATA *get_imu_inside_point(void)
{
    return &imu_inside;
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == INT1_ACCEL_Pin)     //加速度计中断
    {
			Accel_OK=1;
    }
    else if(GPIO_Pin == INT1_GYRO_Pin)    //陀螺仪中断
    {
      Gyro_OK=1;
    }
		else if(GPIO_Pin == GPIO_PIN_0)
    {
        //唤醒任务
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        {
            static BaseType_t xHigherPriorityTaskWoken;
            vTaskNotifyGiveFromISR(INS_task_local_handler, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

















