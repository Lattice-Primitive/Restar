#ifndef KEYBOARD_H
#define KEYBOARD_H

#include "struct_typedef.h"
#include "remote_control.h"


//按键消抖时间			ms
#define KEY_SHAKE_TIME	20

#define PRESS_ON					1
#define PRESS_OFF					0

#define KEY_PRESSED_TIME_Mouse_l    			10
#define KEY_PRESSED_TIME_Mouse_r					10
#define KEY_PRESSED_TIME_Q								10


#define KEY_PRESSED_TIME_W          	 		10
#define KEY_PRESSED_TIME_S								10
#define KEY_PRESSED_TIME_A								10
#define KEY_PRESSED_TIME_D								10

#define KEY_PRESSED_TIME_B								250
#define KEY_PRESSED_TIME_G								250
#define KEY_PRESSED_TIME_X								250

#define KEY_PRESSED_TIME_CTRL							250
#define KEY_PRESSED_TIME_SHIFT						250

#define KEY_PRESSED_TIME_E								10
#define KEY_PRESSED_TIME_R								10
#define KEY_PRESSED_TIME_F								10
#define KEY_PRESSED_TIME_Z								10
#define KEY_PRESSED_TIME_C								10
#define KEY_PRESSED_TIME_V								250


//标志位长度
#define FLAG_SIZE													10+6
typedef enum{
		NO_DOWN = 0,			//没有按下
		DOWN_SHORT,				//短按
		DOWN_LONG					//长按
} keyboard_mode_t;



typedef struct{
		keyboard_mode_t mode;
		uint32_t start_time;
		uint32_t last_time;
		uint32_t hold_time;
		uint32_t pressed_time;
} keyboard_state_t;


typedef struct{
		keyboard_state_t Mouse_l;
		keyboard_state_t Mouse_r;
		keyboard_state_t Q;
	
		keyboard_state_t W;
		keyboard_state_t S;
		keyboard_state_t A;
		keyboard_state_t D;
		keyboard_state_t B;
		keyboard_state_t G;
		keyboard_state_t X;
	
		keyboard_state_t KEY_CTRL;
		keyboard_state_t KEY_SHIFT;
	
		keyboard_state_t S1_L_UP;
		keyboard_state_t S1_L_MID;
		keyboard_state_t S1_L_DOWN;	
	
		keyboard_state_t S0_R_UP;	
		keyboard_state_t S0_R_MID;
		keyboard_state_t S0_R_DOWN;

//new
		keyboard_state_t E;
		keyboard_state_t R;
		keyboard_state_t F;

		keyboard_state_t z;

		keyboard_state_t c;
		keyboard_state_t v;

//	key_data_t CH4_Z;      //遥控器通道4旋转标志  正值			大于+300
//	key_data_t CH4_F;      //遥控器通道4旋转标志  负值			小于-300
	
} keyboard_pressed_t;             		//按键按下标志

typedef struct{
		const RC_ctrl_t *keyboard_RC;               //遥控器指针, remote control
	
		_Bool keyboard_control_flag; //遥控器控制标志 		0关 					1开
	
		_Bool forward_flag;					 //前进标志					0关						1开
		_Bool backward_flag;					 //后退标志					0关						1开
		_Bool left_flag;					 //左横移标志				0关						1开
		_Bool right_flag;					 //右横移标志				0关						1开	
	
//		_Bool chassis_follow_gimbal_flag;        //底盘模式 					0底盘跟随云台 	1底盘不跟随云台
		_Bool gyroscope_flag;            //陀螺模式标志 			0关 					1开
	
		_Bool turn_flag;           //掉头标志 					0关 					1开

		_Bool slow_speed_flag;     //慢速标志 					0关 					1开	
	
		_Bool friction_flag; 			 //摩擦轮开启标志  	0关 					1开
		_Bool shoot_flag;   			 //射击标志     			0关 					1开
		_Bool shoot_continue_flag;   			 //射击标志     			0关 					1开
	
		_Bool clear_flag;   			 //射击标志     			0关 					1开


		_Bool keyboard_E_flag;
		_Bool keyboard_R_flag;
		_Bool keyboard_F_flag;
		_Bool keyboard_Z_flag;
		_Bool keyboard_C_flag;
		_Bool keyboard_V_flag;
		_Bool keyboard_M_R_flag;
		
		_Bool keyboard_SHIFT_L_flag;
		_Bool keyboard_V_SHORT_flag;
		_Bool keyboard_CTRL_L_flag;
		_Bool keyboard_Q_S_flag;
		_Bool keyboard_E_S_flag;
		_Bool keyboard_G_S_flag;
		_Bool UI_open_flag;
} keyboard_ctrl_t;



/**
  * @brief          "keyboard_pressed" valiable initialization, set key pressed time
  * @param[out]     keyboard_pressed_init: "keyboard_pressed" valiable point
  * @retval         none
  */
/**
  * @brief          初始化"keyboard_pressed"变量，设置按键按下时长
  * @param[out]     keyboard_pressed_init:"keyboard_pressed"变量指针.
  * @retval         none
  */
void keyboard_ctrl_init(keyboard_ctrl_t *keyboard_ctrl_init, keyboard_pressed_t *keyboard_pressed_init);
/**
	*@brief 					control data updates
	*@param[in] 			keyboard_ctrl: user control data
	*@param[in] 			keyboard_pressed: key control status
	*@param[in] 			system time: current system time	 		ms
	*@retval
	*/
/**
  * @brief          控制数据更新
  * @param[in]      keyboard_ctrl: 用户控制数据
	* @param[in]      keyboard_pressed: 按键控制状态
  * @param[in]      system_time:当前系统时间    						ms
  * @retval  				       
  */
void keyboard_ctrl_update(keyboard_ctrl_t *keyboard_ctrl, keyboard_pressed_t *keyboard_pressed, uint32_t system_time);


extern keyboard_ctrl_t keyboard_ctrl;	
extern keyboard_pressed_t keyboard_pressed;

#endif 



















