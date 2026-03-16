#include "process_task.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

extern uint8_t flag_Standup;
extern osThreadId_t ProcessHandle;
//步态更改处理，但现在没用上
void Process_Task(void *argument)
{
    while (1)
    {
        if (flag_Standup)
        {
//            Change_NowState(TROT);
            osThreadSuspend(ProcessHandle);
        }
    }
}
