#ifndef __DM8009_DRV_H__
#define __DM8009_DRV_H__
#include "main.h"



#define P_MIN		-3.125f
#define P_MAX		3.125f
#define V_MIN		-45.0f
#define V_MAX		45.0f
#define KP_MIN	0.0f
#define KP_MAX	500.0f
#define KD_MIN 	0.0f
#define KD_MAX	5.0f
#define T_MIN		-54.0f
#define T_MAX		54.0f

typedef struct
{
	uint8_t id;			//控制器ID
	int p_int;			
	int v_int;
	int t_int;
	uint8_t T_MOS;	//驱动上MOS的平均温度			单位：℃
	uint8_t T_Rotor;//电机内部线圈的平均温度   	单位：℃
	float POS;			//位置信息									单位：rad
	float VEL;			//速度信息									单位：rad/s
	float TORQUE;		//扭矩信息									单位：N-m
}DM_J8009_DATA_t;

extern DM_J8009_DATA_t dm_j8009_data[8];		//达妙电机反馈数据

/**
  * @brief    清除错误  发送给达妙电机  DM-J8009
  * @param[in] 	id：	电机ID
 */
extern void CLEAR_ERROR_DM_J8009(int16_t id);

/**
  * @brief   电机使能  发送给达妙电机  DM-J8009
  * @param[in] 	id：	电机ID
 */
extern void ENABLE_DM_J8009(CAN_HandleTypeDef can,int16_t id);


/**
  * @brief电机失能  发送给达妙电机  DM-J8009
  * @param[in] 	id：	电机ID
 */
extern void DISABLE_DM_J8009(CAN_HandleTypeDef can,int16_t id);

/**
  * @brief    保存零点  发送给达妙电机  DM-J8009
  * @param[in]  id：	电机ID
 */
extern void SAVE_ZERO_DM_J8009(CAN_HandleTypeDef can,int16_t id);


/**
  * @brief  MIT模式控制  发送给达妙电机  DM-J8009
  * @param[in]	     id：	电机ID
  * @param[in]		_pos：位置					单位：rad
  * @param[in]		_vel：速度					单位：rad/s
  * @param[in]		_KP：	KP
  * @param[in]		_KD：	KD
  * @param[in]			_torq：转矩				单位：N-m
解析：（id除外）给定位置、KP、KD且其余项为0时，此时为位置控制；给定速度、KD且其余项为0时，此时为速度控制；给定扭矩且其余项为0时，此时为扭矩控制
 */
extern void CTRL_DM_J8009_MIT(CAN_HandleTypeDef can,uint16_t id, float _pos, float _vel, float _KP, float _KD, float _torq);


/**
  * @brief    电机反馈数据  来自达妙电机  DM-J8009
  * @param[in]		    num：	序号
  * @param[in]			data：传入数据
 */
extern void FEEDBACK_DATA_DM_J8009(uint16_t num, uint8_t data[]);










extern float uint_to_float(int x_int, float x_min, float x_max, int bits);
extern int float_to_uint(float x_float, float x_min, float x_max, int bits);

#endif /* __DM8009_DRV_H__ */
