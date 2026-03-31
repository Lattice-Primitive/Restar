#include "keycontrol.h"
#include "servo_control.h"
#include "stdlib.h"
#include "math.h"
#include "cmsis_os.h"
#include "CAN_receive.h"
#include "string.h"
#include "bsp_pump.h"

extern motor_measure_t motor_can1_data[8];    	//can1电机返回数据
extern motor_measure_t motor_can2_data[8];			//can2电机返回数据

_Bool flag_x_r,flag_x_f,flag_sp_l,flag_sp_r;
chassis_speed_t chassis_spd;
spd_limit_analog_switch_t limit_switch,limit_xaxle,limit_smallp;
flag_switch_t flag_switch,flag_xaxle,flag_smallp;

static void motor_sports(void);
static void pitch_roll_sports(void);
static void conv_sports(void);
static void lift_sports(void);
static void extend_sports(void);
static void pump_control(void);

static void speed_limit_f(void);
static void x_axle_limit(void);
static void small_plimit(void);


void key_control_mode(void)
{
	if(keyboard_ctrl.keyboard_control_flag)
	{
		servo_control_gimbal();
		motor_sports();
		lift_sports();
		conv_sports();
		extend_sports();
		pump_control();
		pitch_roll_sports();
		
	}
	else
	{
		chassis_spd.vx_set = map(rc_ctrl.rc.ch[1],-660,660,-2000,2000);
		chassis_spd.vy_set = map(rc_ctrl.rc.ch[0],-660,660,-2000,2000);
		chassis_spd.wz_set = map(rc_ctrl.rc.ch[2],-660,660,-2000,2000);
		if(switch_is_up(rc_ctrl.rc.s[1]))
			pump_on();
		else
			pump_off();
	}
}
//初始化底盘
void first_init(void)
{
	const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
  const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
	const static fp32 chassis_z_order_filter[1] = {CHASSIS_ACCEL_Z_NUM};
	first_order_filter_init(&chassis_spd.vx_fir,CHASSIS_CONTROL_TIME,chassis_x_order_filter);
	first_order_filter_init(&chassis_spd.vy_fir,CHASSIS_CONTROL_TIME,chassis_y_order_filter);
	first_order_filter_init(&chassis_spd.wz_fir,CHASSIS_CONTROL_TIME_WZ,chassis_z_order_filter);
}
//电机运动
static void motor_sports(void)
{
	if(keyboard_ctrl.forward_flag){
			first_order_filter_cali(&chassis_spd.vx_fir,CHASSIS_SPEED);
			chassis_spd.vx_set = chassis_spd.vx_fir.out;
		}
		else if(keyboard_ctrl.backward_flag){
			first_order_filter_cali(&chassis_spd.vx_fir,-CHASSIS_SPEED);
			chassis_spd.vx_set = chassis_spd.vx_fir.out;
		}
		else
			chassis_spd.vx_set = 0;
		
		if(keyboard_ctrl.left_flag){
			first_order_filter_cali(&chassis_spd.vy_fir,-CHASSIS_SPEED);
			chassis_spd.vy_set = chassis_spd.vy_fir.out;
		}
		else if(keyboard_ctrl.right_flag){
			first_order_filter_cali(&chassis_spd.vy_fir,CHASSIS_SPEED);
			chassis_spd.vy_set = chassis_spd.vy_fir.out;
		}
		else
			chassis_spd.vy_set = 0;
				
		chassis_spd.wz_set = map(rc_ctrl.mouse.x,-WZ_MOUSE,WZ_MOUSE,-WZ_SPEED,WZ_SPEED);
}
//传送带
static void conv_sports(void)
{
	if(!keyboard_ctrl.keyboard_V_flag)
	{
		if(keyboard_ctrl.keyboard_V_SHORT_flag)
			enginer_move_can1[5].motor.target_speed = CONV_SPEED;
		else
			enginer_move_can1[5].motor.target_speed = 0;
	}
	else
	{
		enginer_move_can1[5].motor.target_speed = -CONV_SPEED;
	}
}
//抬升
static void lift_sports(void)
{
	//shift切换 一级抬升 二级抬升 X轴
		if(!keyboard_ctrl.keyboard_SHIFT_L_flag){
			enginer_move_can1[4].motor.target_speed = 0;
		
			speed_limit_f();
			//一级抬升
			if(keyboard_ctrl.keyboard_R_flag && !flag_switch.motor_run_flag){
				enginer_move_can1[6].motor.target_speed = -FIRST_SPEED; 
				enginer_move_can1[7].motor.target_speed =  FIRST_SPEED;
				
				enginer_move_can2[5].motor.target_speed = -SECOND_SPEED; 
				enginer_move_can2[6].motor.target_speed =  SECOND_SPEED;
			}
			else if(keyboard_ctrl.keyboard_F_flag){
				enginer_move_can1[6].motor.target_speed =  FIRST_SPEED; 
				enginer_move_can1[7].motor.target_speed = -FIRST_SPEED;
				
				enginer_move_can2[5].motor.target_speed =  SECOND_SPEED; 
				enginer_move_can2[6].motor.target_speed = -SECOND_SPEED;
			}
			else{
				enginer_move_can1[6].motor.target_speed = 0; 
				enginer_move_can1[7].motor.target_speed = 0;
				enginer_move_can2[5].motor.target_speed = 0; 
				enginer_move_can2[6].motor.target_speed = 0;
			}	
		}
		else{
			enginer_move_can1[6].motor.target_speed = 0; 
			enginer_move_can1[7].motor.target_speed = 0;
			enginer_move_can2[5].motor.target_speed = 0; 
			enginer_move_can2[6].motor.target_speed = 0;
			x_axle_limit();
			if(keyboard_ctrl.keyboard_R_flag && !flag_x_r)
				enginer_move_can1[4].motor.target_speed = -X_AXLE_SPEED;
			else if(keyboard_ctrl.keyboard_F_flag && !flag_x_f) 
				enginer_move_can1[4].motor.target_speed =  X_AXLE_SPEED;
			else
				enginer_move_can1[4].motor.target_speed = 0;
		}
}

static void pitch_roll_sports(void)
{
		//ctrl长按切换 大Pitch 小Pitch
		if(!keyboard_ctrl.keyboard_CTRL_L_flag){
			enginer_move_can2[0].motor.target_speed = 0;
			enginer_move_can2[1].motor.target_speed = 0;
			//大P轴
			if(keyboard_ctrl.shoot_continue_flag)
				enginer_move_can2[3].motor.target_speed = -BIG_P_SPEED;
			else if(keyboard_ctrl.keyboard_M_R_flag)
				enginer_move_can2[3].motor.target_speed = BIG_P_SPEED;
			else
				enginer_move_can2[3].motor.target_speed = 0;
			
			//大r
			if(keyboard_ctrl.keyboard_Z_flag)
				enginer_move_can2[2].motor.target_speed = BIG_R_SPEED;
			else if(keyboard_ctrl.keyboard_C_flag)
				enginer_move_can2[2].motor.target_speed = -BIG_R_SPEED;
			else
				enginer_move_can2[2].motor.target_speed = 0;
		}
		else{
			enginer_move_can2[2].motor.target_speed = 0;
			enginer_move_can2[3].motor.target_speed = 0;
			//小P轴
			small_plimit();
			if(keyboard_ctrl.shoot_continue_flag && !flag_sp_l){
				enginer_move_can2[0].motor.target_speed = -SMALL_P_SPEED;
				enginer_move_can2[1].motor.target_speed =  SMALL_P_SPEED;
			}	
			else if(keyboard_ctrl.keyboard_M_R_flag && !flag_sp_r){
				enginer_move_can2[0].motor.target_speed =  SMALL_P_SPEED;
				enginer_move_can2[1].motor.target_speed = -SMALL_P_SPEED;
			}
			//小r
			else if(keyboard_ctrl.keyboard_Z_flag){
				enginer_move_can2[0].motor.target_speed = SMALL_R_SPEED;
				enginer_move_can2[1].motor.target_speed = SMALL_R_SPEED;
			}
			else if(keyboard_ctrl.keyboard_C_flag){
				enginer_move_can2[0].motor.target_speed = -SMALL_R_SPEED;
				enginer_move_can2[1].motor.target_speed = -SMALL_R_SPEED;
			}
			else{
				enginer_move_can2[0].motor.target_speed = 0;
				enginer_move_can2[1].motor.target_speed = 0;
			}
		}
}

static void extend_sports(void)
{
	if(keyboard_ctrl.friction_flag)
		enginer_move_can2[4].motor.target_speed = -EXTEND_SPEED;
	else if(keyboard_ctrl.keyboard_E_flag)
		enginer_move_can2[4].motor.target_speed =  EXTEND_SPEED;
	else
		enginer_move_can2[4].motor.target_speed = 0;
}

static void pump_control(void)
{
	if(keyboard_ctrl.turn_flag)
		pump_on();
	else
		pump_off();
}

static void speed_limit_f(void)
{
	static uint32_t system_time;
	system_time = xTaskGetTickCount();
	limit_switch.cal_rpm_temp   = abs(motor_can1_data[6].speed_rpm); //反馈转速绝对值 rpm
	limit_switch.cal_speed_temp = abs(enginer_move_can1[6].motor.target_speed); //速度绝对值
	if(enginer_move_can1[6].motor.target_speed != 0)
	{
		if(limit_switch.error_speed == limit_switch.last_error_speed)
		{
			limit_switch.error_speed = limit_switch.cal_speed_temp - limit_switch.cal_rpm_temp;
			limit_switch.start_time = system_time;
		}
		else
		{
			limit_switch.last_time = system_time;
			limit_switch.hold_time = fabs(limit_switch.last_time - limit_switch.start_time);
			limit_switch.derror = (limit_switch.error_speed - limit_switch.last_error_speed) / limit_switch.hold_time;
			limit_switch.derror_abs = fabs(limit_switch.derror);
			
			if(limit_switch.derror_abs > WAVE_PEAK_LIMIT && !flag_switch.first_curve_record) 
			{
				flag_switch.first_curve_record = FLAG_FINISH;
			}
			else if(flag_switch.first_curve_record && limit_switch.derror_abs < WAVE_FIRST_FLAG)
			{
				flag_switch.first_curve_flag = FLAG_FINISH;
			}
			limit_switch.last_error_speed = limit_switch.error_speed;
		}
		if(flag_switch.first_curve_flag){
			if(limit_switch.derror_abs > WAVE_RUN_FLAG)
				flag_switch.motor_run_flag = FLAG_FINISH;
		}			
	}
	else
	{
		memset(&limit_switch,0,sizeof(limit_switch));
		flag_switch.first_curve_record = FLAG_START;
		flag_switch.first_curve_flag = FLAG_START;
	}
	if(keyboard_ctrl.keyboard_F_flag)
		flag_switch.motor_run_flag = FLAG_START;
}

static void x_axle_limit(void)
{
	static uint32_t system_time;
	system_time = xTaskGetTickCount();
	limit_xaxle.cal_rpm_temp   = abs(motor_can1_data[4].speed_rpm); //反馈转速绝对值 rpm
	limit_xaxle.cal_speed_temp = abs(enginer_move_can1[4].motor.target_speed); //速度绝对值
	if(enginer_move_can1[4].motor.target_speed != 0)
	{
		if(limit_xaxle.error_speed == limit_xaxle.last_error_speed)
		{
			limit_xaxle.error_speed = limit_xaxle.cal_speed_temp - limit_xaxle.cal_rpm_temp;
			limit_xaxle.start_time = system_time;
		}
		else
		{
			limit_xaxle.last_time = system_time;
			limit_xaxle.hold_time = fabs(limit_xaxle.last_time - limit_xaxle.start_time);
			limit_xaxle.derror = (limit_xaxle.error_speed - limit_xaxle.last_error_speed) / limit_xaxle.hold_time;
			limit_xaxle.derror_abs = fabs(limit_xaxle.derror);
			
			if(limit_xaxle.derror_abs > WAVE_XAXLE_LIMIT && !flag_xaxle.first_curve_record) 
			{
				flag_xaxle.first_curve_record = FLAG_FINISH;
			}
			else if(flag_xaxle.first_curve_record && limit_xaxle.derror_abs < WAVE_X_FST_FLAG)
			{
				flag_xaxle.first_curve_flag = FLAG_FINISH;
			}
			limit_xaxle.last_error_speed = limit_xaxle.error_speed;
		}
		if(flag_xaxle.first_curve_flag){
			if(limit_xaxle.derror_abs > WAVE_XAXLE_FLAG){
				if(keyboard_ctrl.keyboard_R_flag)
					flag_x_r = FLAG_FINISH;
				else if(keyboard_ctrl.keyboard_F_flag)
					flag_x_f = FLAG_FINISH;
			}
		}			
	}
	else
	{
		memset(&limit_xaxle,0,sizeof(limit_xaxle));
		flag_xaxle.first_curve_record = FLAG_START;
		flag_xaxle.first_curve_flag = FLAG_START;
	}
	if(keyboard_ctrl.keyboard_F_flag)
		flag_x_r = FLAG_START;
	if(keyboard_ctrl.keyboard_R_flag)
		flag_x_f = FLAG_START;
}


static void small_plimit(void)
{
	static uint32_t system_time;
	system_time = xTaskGetTickCount();
	limit_smallp.cal_rpm_temp   = abs(motor_can2_data[0].speed_rpm); //反馈转速绝对值 rpm
	limit_smallp.cal_speed_temp = abs(enginer_move_can2[0].motor.target_speed); //速度绝对值
	if(enginer_move_can2[0].motor.target_speed != 0)
	{
		if(limit_smallp.error_speed == limit_smallp.last_error_speed)
		{
			limit_smallp.error_speed = limit_smallp.cal_speed_temp - limit_smallp.cal_rpm_temp;
			limit_smallp.start_time  = system_time;
		}
		else
		{
			limit_smallp.last_time = system_time;
			limit_smallp.hold_time = fabs(limit_smallp.last_time - limit_smallp.start_time);
			limit_smallp.derror = (limit_smallp.error_speed - limit_smallp.last_error_speed) / limit_smallp.hold_time;
			limit_smallp.derror_abs = fabs(limit_smallp.derror);
			
			if(limit_smallp.derror_abs > 110 && !flag_smallp.first_curve_record) 
			{
				flag_smallp.first_curve_record = FLAG_FINISH;
			}
			else if(flag_smallp.first_curve_record && limit_smallp.derror_abs < 50)
			{
				flag_smallp.first_curve_flag = FLAG_FINISH;
			}
			limit_smallp.last_error_speed = limit_smallp.error_speed;
		}
		if(flag_smallp.first_curve_flag){
			if(limit_smallp.derror_abs > 45){
				if(keyboard_ctrl.shoot_continue_flag)
					flag_sp_l = FLAG_FINISH;
				else if(keyboard_ctrl.keyboard_M_R_flag)
					flag_sp_r = FLAG_FINISH;
			}
		}			
	}
	else
	{
		memset(&limit_smallp,0,sizeof(limit_smallp));
		flag_smallp.first_curve_record = FLAG_START;
		flag_smallp.first_curve_flag = FLAG_START;
	}
	if(keyboard_ctrl.shoot_continue_flag)
		flag_sp_r = FLAG_START;
	if(keyboard_ctrl.keyboard_M_R_flag)
		flag_sp_l = FLAG_START;
}












