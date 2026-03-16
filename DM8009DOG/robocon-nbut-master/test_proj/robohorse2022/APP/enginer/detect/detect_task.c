#include "detect_task.h"
#include "drv_led.h"


#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"


uint8_t High_Temp_Warning = 0; //高温警报标志

/**
 * 系统检测任务，用于检测电机温度
 * 流程：检测电机温度->状态灯亮
 * */
void Detect_Task(void *argument)
{
//    while (1)
//    {
//        if (High_Temp_Warning == 0)
//        {
//            for (int i = 0; i < 8; i++)
//            {
//                if (M3508[i].Temp >= 100) //温度达到目标时高温警告
//                    High_Temp_Warning = 1;
//            }
//            LED_On(LED_GREEN);//温度正常绿灯闪烁
//            osDelay(500);
//            LED_Off(LED_GREEN);
//            osDelay(500);
//        }
//        else
//        {
//            LED_On(LED_RED);//高温警告红灯闪烁
//            osDelay(500);
//            LED_Off(LED_RED);
//            osDelay(500);
//        }
//    }
}
