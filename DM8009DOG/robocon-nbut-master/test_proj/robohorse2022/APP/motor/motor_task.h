#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

#include "pid.h"

void Motor_Task(void *argument);
void Motor_Set_MaxSpeed(PID_t *pid,uint16_t speed);

#endif /*MOTOR_TASK_H*/
