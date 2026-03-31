#include "power_limit_pid.h"
#include "pid.h"
#include "referee.h"
#include "math.h"
#include "enginer_task.h"

#define LIMIT_KP 15.0f
#define LIMIT_KI 0.0f
#define LIMIT_KD 0.0f
#define LIMIT_MAX_OUT 1.0f
#define LIMIT_MAX_IOUT 1.0f

#define NO_JUDGE_TOTAL_CURRENT_LIMIT    64000.0f 


fp32 chassis_power_limit,energy;
fp32 power_set = 80;
fp32 limit_factor;
pid_type_def limit_pid;
fp32 Chassis_Limit_PID[3] = {LIMIT_KP,LIMIT_KI,LIMIT_KD};


void Chassis_Power_Cal(void)
{
//	get_chassis_power_and_buffer(&chassis_power_limit,&energy);
//	PID_init(&limit_pid,PID_POSITION,Chassis_Limit_PID,LIMIT_MAX_OUT,LIMIT_MAX_IOUT);
//	PID_clear(&limit_pid);
//	limit_factor = PID_calc(&limit_pid,chassis_power_limit,power_set);
//	enginer_move_can1[0].motor_speed_pid.out*=limit_factor;
//	enginer_move_can1[1].motor_speed_pid.out*=limit_factor;
//	enginer_move_can1[2].motor_speed_pid.out*=limit_factor;
//	enginer_move_can1[3].motor_speed_pid.out*=limit_factor;
	fp32 total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
  fp32 total_current = 0.0f;
	
	for(uint8_t i = 0; i < 4; i++)
    {
        total_current += fabs(enginer_move_can1[i].motor_speed_pid.out);
    }
    
	if(total_current > total_current_limit)
	{
			fp32 current_scale = total_current_limit / total_current;
			enginer_move_can1[0].motor_speed_pid.out*=current_scale;
			enginer_move_can1[1].motor_speed_pid.out*=current_scale;
			enginer_move_can1[2].motor_speed_pid.out*=current_scale;
			enginer_move_can1[3].motor_speed_pid.out*=current_scale;
	}
}











