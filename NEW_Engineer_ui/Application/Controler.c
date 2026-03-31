#include "Controler.h"
#include "main.h"
#include "cmsis_os.h"


#define first_lift_angle_limit 4.5f//一级抬升最高高度
#define second_lift_angle_limit 6.0f//一+二级抬升最高高度

uint8_t flag_w = 0;
//uint8_t ma_flag1 = 0;
int16_t error;

void con_init(void)
{
	enginer_move_can2[2].motor.give_current = 1600;
	flag_w = 1;
//	ma_flag1 = 1;
	osDelay(2000);
	while(1)
	{
		error = enginer_move_can2[2].motor.enginer_motor_measure->ecd - enginer_move_can2[2].motor.enginer_motor_measure->last_ecd;
		if((error < 6) && (error > -6)){
			enginer_move_can2[2].motor.give_current = 0;
			flag_w = 3;
			break;
//			ma_flag1 = 0;
		}
	}
}

void Big_pitch_control(float Big_P_angle)
{
	enginer_move_can2[2].motor.target_angle = Big_P_angle;
}

void Small_pitch_control(float Small_p_angle)
{
	fp32 buf,count;
	buf = -Small_p_angle;
	count += buf;
	if(count > 1.57f)   //需测量角度数据
		buf = 0;
	LimitMax(buf,1.5f)
	enginer_move_can2[3].motor.target_angle = buf;
}

void yaw_control(float y_angle)
{
	enginer_move_can2[4].motor.target_angle = y_angle;
}

void roll_control(float r_angle)
{
	enginer_move_can2[5].motor.target_angle = r_angle;
}

void lift_control(enginer_control_t* lift,enginer_control_t* lift2,float lift_angle)
{
	uint8_t i = 0;
	if(lift_angle > 2.0f)
	{	
		i = 1;
		if(i == 1){
			lift->motor.target_angle = 1000;
			lift2->motor.target_angle = -1000;
			osDelay(2000);
			lift->motor.target_angle = 0;
			lift2->motor.target_angle = -0;
			i++;
		}
	}
		
	
//		enginer_move_can1[4].motor.target_angle = lift_angle;//一级抬升
//	  enginer_move_can1[5].motor.target_angle = -lift_angle;//一级抬升
	
//	if(lift_angle < first_lift_angle_limit){

//		
//		enginer_move_can2[0].motor.target_angle = 0;//二级抬升
//	  enginer_move_can2[1].motor.target_angle = 0;//二级抬升
//	}
//	else if(lift_angle > first_lift_angle_limit && lift_angle < second_lift_angle_limit){
//		enginer_move_can1[4].motor.target_angle = first_lift_angle_limit;//一级抬升
//	  enginer_move_can1[5].motor.target_angle = -first_lift_angle_limit;//一级抬升
//		const float diff = lift_angle - first_lift_angle_limit;
//		enginer_move_can2[0].motor.target_angle = diff;//二级抬升
//	  enginer_move_can2[1].motor.target_angle = diff;//二级抬升
//	}
}

void forward_control(float forward_angle)
{
	enginer_move_can1[6].motor.target_angle = forward_angle;
}


void ALL_control(float set_angle)
{
//	Big_pitch_control(set_angle);
//	Small_pitch_control(set_angle);
//	yaw_control(set_angle);
//	roll_control(set_angle);
//	lift_control(set_angle);
//	forward_control(set_angle);
}


