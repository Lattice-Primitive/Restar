/**
  ****************************(C) COPYRIGHT 2024 R&A战队****************************
  * @file       keyboard.c/h
  * @brief      键盘功能定义.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2024-03-13     	TAN             1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 R&A战队****************************
  */


#include "keyboard.h"
#include "CAN_receive.h"
#include "string.h"


keyboard_ctrl_t keyboard_ctrl;					//keyboard control data, 按键控制数据
keyboard_pressed_t keyboard_pressed;		//keyboard state, 按键按下状态

/**
  * @brief 					detect key status
	* @param[in] 			pressed_state: key status
  * @param[in] 			key_state: key status pointer
  * @param[in] 			system_time: current system time		ms
  * @retval         none
  */
/**
  * @brief          检测按键状态
	* @param[in]      pressed_state: 按键状态
  * @param[in]      key_state: 按键状态指针
  * @param[in]      system_time: 当前系统时间    				ms
  * @retval         none
  */
static void detect_key_state(_Bool pressed_state, keyboard_state_t *key_state, uint32_t system_time);
/**
  * @brief 					keyboard control updated
	* @param[in] 			keyboard_ctrl: user controls the data pointer
	* @param[in] 			keyboard_pressed_update: the keyboard detects the pointer
  * @retval         none
  */
/**
  * @brief          键盘控制更新
	* @param[in]      keyboard_ctrl: 用户控制数据指针
	* @param[in]      keyboard_pressed_update_update: 键盘检测指针
  * @retval         none
  */
static void keyboard_pressed_update(keyboard_ctrl_t *keyboard_ctrl, keyboard_pressed_t *keyboard_pressed_update);



/**
  * @brief 					detect key status
	* @param[in] 			pressed_state: key status
  * @param[in] 			key_state: key status pointer
  * @param[in] 			system_time: current system time		ms
  * @retval         none
  */
/**
  * @brief          检测按键状态
	* @param[in]      pressed_state: 按键状态
  * @param[in]      key_state: 按键状态指针
  * @param[in]      system_time: 当前系统时间    				ms
  * @retval         none
  */
static void detect_key_state(_Bool pressed_state, keyboard_state_t *key_state, uint32_t system_time)
{
		if (pressed_state)
		{
				//determine whether it is the first time to press
				//判断是否为首次按下
				if (key_state->start_time == key_state->last_time)
				{		
						//record start time
						//记录开始时间
						key_state->start_time = system_time;			
				}
				else
				{
						key_state->hold_time = system_time - key_state->start_time;
						if (key_state->hold_time < KEY_SHAKE_TIME)
						{	
								return;
						}
						else if (key_state->hold_time > key_state->pressed_time)
						{
								key_state->mode = DOWN_LONG;
						}
				}
		}
		else
		{
				if(key_state->hold_time > KEY_SHAKE_TIME && key_state->hold_time < key_state->pressed_time)
				{
						key_state->mode = DOWN_SHORT;
				}
				else
				{
						key_state->mode = NO_DOWN;
				}
				
				key_state->start_time = system_time;
				key_state->last_time = key_state->start_time;
				key_state->hold_time = 0;
		}
}

/**
  * @brief 					keyboard control updated
	* @param[in] 			keyboard_ctrl: user controls the data pointer
	* @param[in] 			keyboard_pressed_update: the keyboard detects the pointer
  * @retval         none
  */
/**
  * @brief          键盘控制更新
	* @param[in]      keyboard_ctrl_update: 用户控制数据指针
	* @param[in]      keyboard_pressed_update: 键盘检测指针
  * @retval         none
  */
static void keyboard_pressed_update(keyboard_ctrl_t *keyboard_ctrl_update, keyboard_pressed_t *keyboard_pressed_update)  
{	
		//mouse:left			press time:short press										shoot: once 									switch: ON/OFF
		//按键：鼠标左键		按下时长：短按															射击模式：单次									开关量	：ON/OFF	
		if(keyboard_pressed_update->Mouse_l.mode == DOWN_SHORT) 
		{
				keyboard_ctrl_update->shoot_flag = PRESS_ON;
		}
		else if(keyboard_pressed_update->Mouse_l.mode == NO_DOWN) 
		{
				keyboard_ctrl_update->shoot_flag = PRESS_OFF;
		}	
		//mouse:left		  press time:long press											shoot: continue 							switch: ON/OFF
		//按键：鼠标左键		按下时长：长按															射击模式：连发									开关量	：ON/OFF			
		if(keyboard_pressed_update->Mouse_l.mode == DOWN_LONG) 
		{
				keyboard_ctrl_update->shoot_continue_flag = PRESS_ON; 
		}
		else if(keyboard_pressed_update->Mouse_l.mode == NO_DOWN) 
		{
				keyboard_ctrl_update->shoot_continue_flag = PRESS_OFF;
		}		
		//keyboard:Q			press time:short press/long press					shoot: open fric 	  						switch: ON/OFF
		//按键：Q					按下时长：短按/长按												射击模式：开启摩擦轮							开关量	：ON/OFF		
		if (keyboard_pressed_update->Q.mode == DOWN_SHORT ||keyboard_pressed_update->Q.mode == DOWN_LONG) 
		{     
				keyboard_ctrl_update->friction_flag = PRESS_ON;
		}
		else
		{
				keyboard_ctrl_update->friction_flag = PRESS_OFF;
		}
//		if(keyboard_pressed_update->Q.mode == DOWN_SHORT)
//		{
//			    keyboard_ctrl_update->keyboard_Q_S_flag = !keyboard_ctrl_update->keyboard_Q_S_flag;
//		}
		
		
		//keyboard:W			press time:short press/long press					move: forward 									switch: ON/OFF
		//按键：W					按下时长：短按/长按												运动方式：前进										开关量	：ON/OFF
		if (keyboard_pressed_update->W.mode == DOWN_SHORT ||keyboard_pressed_update->W.mode == DOWN_LONG)
		{
				keyboard_ctrl_update->forward_flag = PRESS_ON;
		}
		else
		{
				keyboard_ctrl_update->forward_flag = PRESS_OFF;
		}
		//keyboard:S			press time:short press/long press					move: backward 									switch: ON/OFF
		//按键：S					按下时长：短按/长按												运动方式：后退										开关量	：ON/OFF
		if (keyboard_pressed_update->S.mode == DOWN_SHORT || keyboard_pressed_update->S.mode == DOWN_LONG)
		{
				keyboard_ctrl_update->backward_flag = PRESS_ON;
		}
		else
		{
				keyboard_ctrl_update->backward_flag = PRESS_OFF;
		}
		//keyboard:A			press time:short press/long press					move: left-side transfer 				switch: ON/OFF
		//按键：A					按下时长：短按/长按												运动方式：左横移									开关量	：ON/OFF
		if(keyboard_pressed_update->A.mode == DOWN_SHORT || keyboard_pressed_update->A.mode == DOWN_LONG)
		{
				keyboard_ctrl_update->left_flag = PRESS_ON;
		}
		else
		{
				keyboard_ctrl_update->left_flag = PRESS_OFF;
		}
		//keyboard:D			press time:short press/long press					move: right-side transfer 			switch: ON/OFF
		//按键：D					按下时长：短按/长按												运动方式：右横移									开关量	：ON/OFF
		if(keyboard_pressed_update->D.mode == DOWN_SHORT || keyboard_pressed_update->D.mode == DOWN_LONG)
		{
				keyboard_ctrl_update->right_flag = PRESS_ON;
		}
		else
		{
				keyboard_ctrl_update->right_flag = PRESS_OFF;
		}
		//keyboard:SHIFT	press time:short press/long press					move: slow speed					 			switch: ON/OFF
		//按键：SHIFT			按下时长：短按        										运动方式：静步（慢速）						开关量	：ON/OFF		
		if(keyboard_pressed_update->KEY_SHIFT.mode == DOWN_SHORT)
		{
				keyboard_ctrl_update->slow_speed_flag = !keyboard_ctrl_update->slow_speed_flag;
		}

		if(keyboard_pressed_update->KEY_SHIFT.mode == DOWN_LONG)
		{
			keyboard_ctrl_update->keyboard_SHIFT_L_flag = PRESS_ON;
		}
		else
		{
			keyboard_ctrl_update->keyboard_SHIFT_L_flag = PRESS_OFF;
		}
		//keyboard:B				press time:short press									mode: remote_control								retention value: ON/OFF
		//按键：B						按下时长：短按		  											模式：遥控器控制											点动保持量	：ON/OFF
		if (keyboard_pressed_update->B.mode == DOWN_SHORT)
		{
				keyboard_ctrl_update->keyboard_control_flag = !keyboard_ctrl_update->keyboard_control_flag;
				keyboard_ctrl_update->UI_open_flag = 1;
			
				//圈数清零
				motor_can1_data[4].ecd_count = 0;
				motor_can1_data[6].ecd_count = 0;
		}					
		//keyboard:G				press time:short press									mode: chassis_follow_gimbal					retention value: ON/OFF
		//按键：G						按下时长：短按		  											模式：底盘跟随云台										点动保持量	：ON/OFF
//		if (keyboard_pressed_update->G.mode == DOWN_SHORT || keyboard_pressed_update->G.mode == DOWN_LONG)
//		{
//				keyboard_ctrl_update->chassis_follow_gimbal_flag = PRESS_ON;
//		}					
//		else
//		{
//				keyboard_ctrl_update->chassis_follow_gimbal_flag = PRESS_OFF;
//		}
		if (keyboard_pressed_update->G.mode == DOWN_SHORT)
		{
				keyboard_ctrl_update->keyboard_G_S_flag = !keyboard_ctrl_update->keyboard_G_S_flag;
		}
		//keyboard:X				press time:short press									mode: turn head													retention value: ON/OFF
		//按键：X						按下时长：短按		  											模式：一键掉头														点动保持量	：ON/OFF		
		if (keyboard_pressed_update->X.mode == DOWN_SHORT)
		{
				keyboard_ctrl_update->turn_flag = !keyboard_ctrl_update->turn_flag;
		}		
		
		
		//keyboard:CTRL			press time:short press									mode: gyroscope 										retention value: ON/OFF
		//按键：CTRL					按下时长：短按		  											模式：小陀螺													点动保持量	：ON/OFF
		if(keyboard_pressed_update->KEY_CTRL.mode == DOWN_SHORT) 
		{
				keyboard_ctrl_update->gyroscope_flag = !keyboard_ctrl_update->gyroscope_flag;
		}	
		//按键：CTRL					按下时长：长按
		if(keyboard_pressed_update->KEY_CTRL.mode == DOWN_LONG)
		{
				keyboard_ctrl_update->keyboard_CTRL_L_flag = PRESS_ON;
		}
		else
		{
				keyboard_ctrl_update->keyboard_CTRL_L_flag = PRESS_OFF;
		}
		
		//按键：E
		if(keyboard_pressed_update->E.mode == DOWN_SHORT || keyboard_pressed_update->E.mode == DOWN_LONG)
		{
				keyboard_ctrl_update->keyboard_E_flag = PRESS_ON;
		}
		else
		{
				keyboard_ctrl_update->keyboard_E_flag = PRESS_OFF;
		}
//		if(keyboard_pressed_update->E.mode == DOWN_SHORT)
//		{
//					keyboard_ctrl_update->keyboard_E_S_flag = !keyboard_ctrl_update->keyboard_E_S_flag;
//		}
		
		//按键：R 长按
		if(keyboard_pressed_update->R.mode == DOWN_LONG || keyboard_pressed_update->R.mode == DOWN_SHORT)
		{
				keyboard_ctrl_update->keyboard_R_flag = PRESS_ON;
		}
		else
		{
				keyboard_ctrl_update->keyboard_R_flag = PRESS_OFF;
		}
		
		//按键：F
		if(keyboard_pressed_update->F.mode == DOWN_SHORT || keyboard_pressed_update->F.mode == DOWN_LONG)
		{
				keyboard_ctrl_update->keyboard_F_flag = PRESS_ON;
		}
		else
		{
				keyboard_ctrl_update->keyboard_F_flag = PRESS_OFF;
		}
		
		//按键：Z
		if(keyboard_pressed_update->z.mode == DOWN_SHORT || keyboard_pressed_update->z.mode == DOWN_LONG)
		{
				keyboard_ctrl_update->keyboard_Z_flag = PRESS_ON;
		}
		else
		{
				keyboard_ctrl_update->keyboard_Z_flag = PRESS_OFF;
		}
		
		//按键：C
		if(keyboard_pressed_update->c.mode == DOWN_SHORT || keyboard_pressed_update->c.mode == DOWN_LONG)
		{
				keyboard_ctrl_update->keyboard_C_flag = PRESS_ON;
		}
		else
		{
				keyboard_ctrl_update->keyboard_C_flag = PRESS_OFF;
		}
		
		//按键：V长按
		if(keyboard_pressed_update->v.mode == DOWN_LONG)
		{
				keyboard_ctrl_update->keyboard_V_flag = PRESS_ON;
		}
		else
		{
				keyboard_ctrl_update->keyboard_V_flag = PRESS_OFF;
		}
		//按键：V点按
		if(keyboard_pressed_update->v.mode == DOWN_SHORT)
		{
				keyboard_ctrl_update->keyboard_V_SHORT_flag = !keyboard_ctrl_update->keyboard_V_SHORT_flag;
		}
		
		//鼠标右键
		if(keyboard_pressed_update->Mouse_r.mode == DOWN_SHORT || keyboard_pressed_update->Mouse_r.mode == DOWN_LONG)
		{
				keyboard_ctrl_update->keyboard_M_R_flag = PRESS_ON;
		}
		else
		{
				keyboard_ctrl_update->keyboard_M_R_flag = PRESS_OFF;
		}
}

///**
//  * @brief          用户设置数据
//	* @param[in]      *user_data:用户控制数据
//  * @param[in]      *press_data:按键数据指针    
//  * @retval  				       
//  */
//static void user_set_data(user_costom_data *user_data, press_data_t *press_data){



//		//E 			短按			自瞄模式切换   	点动保持量   		OK
//		if(press_data->E.mode == DOWN_SHORT){
//			set_data.aim_mode_flag = !set_data.aim_mode_flag;
//		}
//		//R  			短按			弹仓开关   			点动保持量   		OK
//		if(press_data->R.mode == DOWN_SHORT){
//			set_data.magazine_flag = !set_data.magazine_flag;
//		}
//		//F 			短按			开关超级电容   	点动保持量   		OK
//		if(press_data->F.mode == DOWN_SHORT){
//			set_data.capacitor_flag = !set_data.capacitor_flag;
//		}
//		//V 			短按长按		摇摆模式   			开关量    				OK
//		if(press_data->V.mode == DOWN_SHORT || press_data->V.mode == DOWN_LONG){
//			set_data.sway_flag = 1;
//		}
//		else{
//			set_data.sway_flag = 0;
//		}
//		//Z 			短按			开关激光   			点动保持量    		OK
//		if(press_data->Z.mode == DOWN_SHORT){
//			set_data.laser_flag = !set_data.laser_flag;
//		}

//		//C 			短按长按		拨弹电机反转   	开关量   				OK		（可考虑去掉）
//		if(press_data->C.mode == DOWN_SHORT || press_data->C.mode == DOWN_LONG){
//			set_data.reverse_flag=1;
//		}
//		else{
//			set_data.reverse_flag=0;
//		}

//		//B 			短按			关闭遥控器摇杆控制及UI显示   	点动保持量  		OK
//		if(press_data->B.mode == DOWN_SHORT){
//			set_data.remote_control_flag = !set_data.remote_control_flag;
//			set_data.UI_open_flag=!set_data.UI_open_flag;
//		}
//		//鼠标右键	长按   		自瞄  					开关量      OK
//		if(press_data->mouse_r.mode == DOWN_LONG){
//			set_data.aim_flag=1;
//		}
//		else{
//			set_data.aim_flag=0;
//		}


//		//遥控器滚轮向左滚  短按长按		射击(单点和连射)		开关量     OK		(仅遥控器控制标志开启时有效) 
//		if((press_data->CH4_Z.mode == DOWN_SHORT || press_data->CH4_Z.mode == DOWN_LONG)&& set_data.remote_control_flag){
//			set_data.shoot_flag=1;
//		}
//		else{
//			set_data.shoot_flag=0;
//		}
//		//遥控器滚轮右滚  	短按长按			拨弹电机增堵反转   开关量   OK
//		if((press_data->CH4_F.mode == DOWN_SHORT || press_data->CH4_F.mode == DOWN_LONG) && set_data.remote_control_flag){
//			set_data.reverse_flag=1;
//			set_data.shoot_flag=0;
//		}
//		else{
//			set_data.reverse_flag=0;
//		}
//		//左拨杆往下  	长按		开弹仓 			开关量  			OK
//		if(press_data->S1_L_DOWN.mode == DOWN_LONG && set_data.remote_control_flag){
//			set_data.magazine_flag=1;
//		}
//		else{
//			set_data.magazine_flag=0;
//		}
//		//左拨杆往上  	长按		开启摩擦轮 	开关量  			OK
//		if(press_data->S1_L_UP.mode == DOWN_LONG && set_data.remote_control_flag ){
//			set_data.friction_flag =1;
//		}
//		else if(set_data.remote_control_flag ){
//			set_data.friction_flag =0;
//		}
//		//右拨杆往下的时候，恢复遥控器控制模式				OK
//		if(press_data->S0_R_DOWN.mode == DOWN_LONG && !set_data.remote_control_flag)
//			set_data.remote_control_flag = 1;
//		

//		
//		
//}

/**
  * @brief          "keyboard_pressed_update" valiable initialization, set key pressed time
  * @param[out]     keyboard_pressed_update_init: "keyboard_pressed_update" valiable point
  * @retval         none
  */
/**
  * @brief          初始化"keyboard_pressed_update"变量，设置按键按下时长
  * @param[out]     keyboard_pressed_update_init: "keyboard_pressed_update"变量指针.
  * @retval         none
  */
void keyboard_ctrl_init(keyboard_ctrl_t *keyboard_ctrl_init, keyboard_pressed_t *keyboard_pressed_init) 
{
		//get control point
		//获取控制指针
		keyboard_ctrl_init->keyboard_RC = get_remote_control_point();	

		//set key pressed time
		//设置按键按下时长
		keyboard_pressed_init->Mouse_l.pressed_time = KEY_PRESSED_TIME_Mouse_l; 
		keyboard_pressed_init->Mouse_r.pressed_time = KEY_PRESSED_TIME_Mouse_r; 
		keyboard_pressed_init->Q.pressed_time = KEY_PRESSED_TIME_Q; 
		
		keyboard_pressed_init->W.pressed_time = KEY_PRESSED_TIME_W;
		keyboard_pressed_init->S.pressed_time = KEY_PRESSED_TIME_S;
		keyboard_pressed_init->A.pressed_time = KEY_PRESSED_TIME_A;
		keyboard_pressed_init->D.pressed_time = KEY_PRESSED_TIME_D;
	
		keyboard_pressed_init->B.pressed_time = KEY_PRESSED_TIME_B; 
		keyboard_pressed_init->X.pressed_time = KEY_PRESSED_TIME_X; 
		keyboard_pressed_init->G.pressed_time = KEY_PRESSED_TIME_G; 
	
		keyboard_pressed_init->KEY_CTRL.pressed_time = KEY_PRESSED_TIME_CTRL;
		keyboard_pressed_init->KEY_SHIFT.pressed_time = KEY_PRESSED_TIME_SHIFT;
		//new
		keyboard_pressed_init->E.pressed_time = KEY_PRESSED_TIME_E; 
		keyboard_pressed_init->R.pressed_time = KEY_PRESSED_TIME_R; 
		keyboard_pressed_init->F.pressed_time = KEY_PRESSED_TIME_F;
		keyboard_pressed_init->z.pressed_time = KEY_PRESSED_TIME_Z;
		keyboard_pressed_init->c.pressed_time = KEY_PRESSED_TIME_C;
		keyboard_pressed_init->v.pressed_time = KEY_PRESSED_TIME_V;
		
}
/**
	*@brief 					control data updates
	*@param[in] 			keyboard_ctrl: user control data
	*@param[in] 			keyboard_pressed_update: key control status
	*@param[in] 			system time: current system time	 		ms
	*@retval
	*/
/**
  * @brief          控制数据更新
  * @param[in]      keyboard_ctrl: 用户控制数据
	* @param[in]      keyboard_pressed_update: 按键控制状态
  * @param[in]      system_time:当前系统时间    						ms
  * @retval  				       
  */
void keyboard_ctrl_update(keyboard_ctrl_t *keyboard_ctrl, keyboard_pressed_t *keyboard_pressed, uint32_t system_time)
{
    if (keyboard_ctrl == NULL)
    {
        return;
    }

		//detect mouse
		//鼠标按键检测	
		detect_key_state(keyboard_ctrl->keyboard_RC->mouse.press_l, &keyboard_pressed->Mouse_l, system_time);
		detect_key_state(keyboard_ctrl->keyboard_RC->mouse.press_r, &keyboard_pressed->Mouse_r, system_time);		
		
		//detect keyboard 
		//键盘按键检测
		detect_key_state(keyboard_ctrl->keyboard_RC->key.v & KEY_PRESSED_OFFSET_Q, &keyboard_pressed->Q, system_time);
		detect_key_state(keyboard_ctrl->keyboard_RC->key.v & KEY_PRESSED_OFFSET_W, &keyboard_pressed->W, system_time);
		detect_key_state(keyboard_ctrl->keyboard_RC->key.v & KEY_PRESSED_OFFSET_S, &keyboard_pressed->S, system_time);
		detect_key_state(keyboard_ctrl->keyboard_RC->key.v & KEY_PRESSED_OFFSET_A, &keyboard_pressed->A, system_time);
		detect_key_state(keyboard_ctrl->keyboard_RC->key.v & KEY_PRESSED_OFFSET_D, &keyboard_pressed->D, system_time);
		detect_key_state(keyboard_ctrl->keyboard_RC->key.v & KEY_PRESSED_OFFSET_B, &keyboard_pressed->B, system_time);
		detect_key_state(keyboard_ctrl->keyboard_RC->key.v & KEY_PRESSED_OFFSET_G, &keyboard_pressed->G, system_time);
		detect_key_state(keyboard_ctrl->keyboard_RC->key.v & KEY_PRESSED_OFFSET_X, &keyboard_pressed->X, system_time);
		//new
		detect_key_state(keyboard_ctrl->keyboard_RC->key.v & KEY_PRESSED_OFFSET_E, &keyboard_pressed->E, system_time);
		detect_key_state(keyboard_ctrl->keyboard_RC->key.v & KEY_PRESSED_OFFSET_R, &keyboard_pressed->R, system_time);
		detect_key_state(keyboard_ctrl->keyboard_RC->key.v & KEY_PRESSED_OFFSET_F, &keyboard_pressed->F, system_time);
		detect_key_state(keyboard_ctrl->keyboard_RC->key.v & KEY_PRESSED_OFFSET_Z, &keyboard_pressed->z, system_time);
		detect_key_state(keyboard_ctrl->keyboard_RC->key.v & KEY_PRESSED_OFFSET_C, &keyboard_pressed->c, system_time);
		detect_key_state(keyboard_ctrl->keyboard_RC->key.v & KEY_PRESSED_OFFSET_V, &keyboard_pressed->v, system_time);
		
		
		detect_key_state(keyboard_ctrl->keyboard_RC->key.v & KEY_PRESSED_OFFSET_CTRL, &keyboard_pressed->KEY_CTRL, system_time);	
		detect_key_state(keyboard_ctrl->keyboard_RC->key.v & KEY_PRESSED_OFFSET_SHIFT, &keyboard_pressed->KEY_SHIFT, system_time);	
		
		//detect remote
		//遥控器按键检测
		detect_key_state(switch_is_up(keyboard_ctrl->keyboard_RC->rc.s[1]), &keyboard_pressed->S1_L_UP, system_time);		
		detect_key_state(switch_is_mid(keyboard_ctrl->keyboard_RC->rc.s[1]), &keyboard_pressed->S1_L_MID, system_time);
		detect_key_state(switch_is_down(keyboard_ctrl->keyboard_RC->rc.s[1]), &keyboard_pressed->S1_L_DOWN, system_time);
		
		detect_key_state(switch_is_up(keyboard_ctrl->keyboard_RC->rc.s[0]), &keyboard_pressed->S0_R_UP, system_time);		
		detect_key_state(switch_is_mid(keyboard_ctrl->keyboard_RC->rc.s[0]), &keyboard_pressed->S0_R_MID, system_time);
		detect_key_state(switch_is_down(keyboard_ctrl->keyboard_RC->rc.s[0]), &keyboard_pressed->S0_R_DOWN, system_time); 

//	key_mode_update(rc->key.v & KEY_PRESSED_OFFSET_Q, &press_data.Q, system_time, 250);
//	key_mode_update(rc->key.v & KEY_PRESSED_OFFSET_E, &press_data.E, system_time, 250);
//	key_mode_update(rc->key.v & KEY_PRESSED_OFFSET_R, &press_data.R, system_time, 250);
//	key_mode_update(rc->key.v & KEY_PRESSED_OFFSET_F, &press_data.F, system_time, 250);
//	key_mode_update(rc->key.v & KEY_PRESSED_OFFSET_G, &press_data.G, system_time, 250);
//	key_mode_update(rc->key.v & KEY_PRESSED_OFFSET_Z, &press_data.Z, system_time, 250);
//	key_mode_update(rc->key.v & KEY_PRESSED_OFFSET_X, &press_data.X, system_time, 250);
//	key_mode_update(rc->key.v & KEY_PRESSED_OFFSET_C, &press_data.C, system_time, 250);
//	key_mode_update(rc->key.v & KEY_PRESSED_OFFSET_V, &press_data.V, system_time, 250);
//	key_mode_update(rc->key.v & KEY_PRESSED_OFFSET_B, &press_data.B, system_time, 250);

//	key_mode_update((rc->rc.ch[4] > 300), &press_data.CH4_Z, system_time, 250);
//	key_mode_update((rc->rc.ch[4] < -300), &press_data.CH4_F, system_time, 250);
	
		//更新键盘按键控制
		keyboard_pressed_update(keyboard_ctrl, keyboard_pressed);
		
		//切换遥控器标志位清零
		if (keyboard_ctrl->keyboard_control_flag)
		{
				if (!keyboard_ctrl->clear_flag)
				{
						memset(&keyboard_ctrl->forward_flag, 0, FLAG_SIZE);
						keyboard_ctrl->clear_flag = 1;
				}
		}
		else
		{
				keyboard_ctrl->clear_flag = 0;
		}
}








