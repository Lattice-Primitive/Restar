/*
******************************更新于2024年3月23日*********************************
参考手册：RoboMaster裁判系统串口协议附录V1.5
更新人：谭启日		2547332441@qq.com
*/


#ifndef REFEREE_H
#define REFEREE_H

#include "main.h"

#define REFEREE_DATA_MIN_SIZE			10  //接收数据的最小长度
#define REF_PROTOCOL_HEADER_SIZE	5		//帧头数据长度
#define REF_PROTOCOL_CMD_SIZE 		2		//命令数据长度
#define REF_PROTOCOL_CRC16_SIZE		2		//CRC16校验数据长度


#pragma pack(push, 1)

//命令ID
typedef enum
{
	GAME_STATUS_CMD_ID                		= 0x0001,
	GAME_RESULT_CMD_ID                		= 0x0002,
	GAME_ROBOT_HP_CMD_ID              		= 0x0003,
	EVENTS_DATA_CMD_ID               			= 0x0101,
	EXT_SUPPLY_PROJECTILE_ACTION_CMD_ID   = 0x0102,
	REFEREE_WARNING_CMD_ID            		= 0x0104,
	DART_REMAINING_TIME_CMD_ID						=	0x0105,
	ROBOT_STATUS_CMD_ID                		= 0x0201,
	POWER_HEAT_DATA_CMD_ID            		= 0x0202,
	ROBOT_POS_CMD_ID                  		= 0x0203,
	ROBOT_BUFF_CMD_ID                  		= 0x0204,
	AIR_SUPPORT_DATA_CMD_ID        				= 0x0205,
	HURT_DATA_CMD_ID                 			= 0x0206,
	SHOOT_DATA_CMD_ID                 		= 0x0207,
	PROJECTILE_ALLOWANCE_CMD_ID           = 0x0208,
	RFID_STATUS_CMD_ID										=	0x0209,
	DART_CLIENT_CMD_ID										=	0x020A,
	GROUND_ROBOT_POSITION_CMD_ID					=	0x020B,
	RADAR_MARK_DATA_CMD_ID								=	0x020C,
	SENTRY_INFO_CMD_ID										=	0x020D,
	RADAR_INFO_CMD_ID											=	0x020E,
	CUSTOM_ROBOT_DATA_CMD_ID							=	0x0302,
	MAP_COMMAND_CMD_ID										=	0x0303,
	REMOTE_CONTROL_CMD_ID									=	0x0304,
	MAP_ROBOT_DATA_CMD_ID									=	0x0305,
	CUSTOM_CLIENT_CATA_CMD_ID							=	0x0306,
	MAP_SENTRY_DATA_CMD_ID								=	0x0307,
	CUSTOM_INFO_CMD_ID										=	0x0308
}referee_cmd_id_t;

//机器人ID
typedef enum
{
	RED_HERO        = 1,
	RED_ENGINEER    = 2,
	RED_STANDARD_1  = 3,
	RED_STANDARD_2  = 4,
	RED_STANDARD_3  = 5,
	RED_AERIAL      = 6,
	RED_SENTRY      = 7,
	RED_DART				=	8,
	RED_RADAR				=	9,
	RED_OUTPOST			=	10,
	RED_BASE				=	11,

	BLUE_HERO       = 101,
	BLUE_ENGINEER   = 102,
	BLUE_STANDARD_1 = 103,
	BLUE_STANDARD_2 = 104,
	BLUE_STANDARD_3 = 105,
	BLUE_AERIAL     = 106,
	BLUE_SENTRY     = 107,
	BLUE_DART				=	108,
	BLUE_RADAR			=	109,
	BLUE_OUTPOST		=	110,
	BLUE_BASE				=	111,
}robot_id_t;

//比赛进度
typedef enum
{
	PROGRESS_UNSTART        = 0,
	PROGRESS_PREPARE        = 1,
	PROGRESS_SELFCHECK      = 2,
	PROGRESS_5sCOUNTDOWN    = 3,
	PROGRESS_BATTLE         = 4,
	PROGRESS_CALCULATING    = 5,
}game_progress_t;

//帧头结构体
typedef  struct
{
	uint8_t SOF;
	uint16_t data_length;
	uint8_t seq;
	uint8_t CRC8;
} frame_header_struct_t;


//0x0001		11		比赛状态数据，固定以 1Hz 频率发送						服务器-->全体机器人    			常规链路
typedef __packed struct
{
	uint8_t game_type : 4;
	uint8_t game_progress : 4;
	uint16_t stage_remain_time;
	uint64_t SyncTimeStamp;
}game_status_t;

//0x0002		1			比赛结果数据，比赛结束触发发送						服务器-->全体机器人    			常规链路
typedef __packed struct
{
	uint8_t winner;
}game_result_t;

//0x0003		32			机器人血量数据，固定以 3Hz 频率发送					服务器-->全体机器人    			常规链路
typedef __packed struct
{
	uint16_t red_1_robot_HP;
	uint16_t red_2_robot_HP;
	uint16_t red_3_robot_HP;
	uint16_t red_4_robot_HP;
	uint16_t red_5_robot_HP;
	uint16_t red_7_robot_HP;
	uint16_t red_outpost_HP;
	uint16_t red_base_HP;
	
	uint16_t blue_1_robot_HP;
	uint16_t blue_2_robot_HP;
	uint16_t blue_3_robot_HP;
	uint16_t blue_4_robot_HP;
	uint16_t blue_5_robot_HP;
	uint16_t blue_7_robot_HP;
	uint16_t blue_outpost_HP;
	uint16_t blue_base_HP;
}game_robot_HP_t;

//0x0101		4			场地事件数据，固定以 1Hz 频率发送						服务器-->己方全体机器人    	常规链路
typedef __packed struct
{
	uint32_t event_data;
}event_data_t;

//0x0102		4			补给站动作标识数据，补给站弹丸释放时触发发送			服务器-->己方全体机器人	    常规链路
typedef __packed struct
{
	uint8_t supply_projectile_id;
	uint8_t supply_robot_id;
	uint8_t supply_projectile_step;
	uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

//0x0104		3			裁判警告数据，己方判罚/判负时触发发送，其余时间以 1Hz 频率发送						服务器→被判罚方全体机器人    	常规链路
typedef __packed struct
{
	uint8_t level;
	uint8_t offending_robot_id;
	uint8_t count;
}referee_warning_t;

//0x0105		3			飞镖发射相关数据，固定以 1Hz频率发送				服务器-->己方全体机器人    	常规链路
typedef __packed struct
{
	uint8_t dart_remaining_time;
	uint16_t dart_info;
}dart_remaining_time_t;

//0x0201		13			机器人性能体系数据，固定以10Hz 频率发送			主控模块-->对应机器人    		常规链路
typedef __packed struct
{
	uint8_t robot_id;
	uint8_t robot_level;
	uint16_t current_HP;
	uint16_t maximum_HP;
	uint16_t shooter_barrel_cooling_value;
	uint16_t shooter_barrel_heat_limit;
	uint16_t chassis_power_limit; 
	uint8_t power_management_gimbal_output : 1;
	uint8_t power_management_chassis_output : 1;
	uint8_t power_management_shooter_output : 1;
	uint8_t reserved:5;
}robot_status_t;

//0x0202		16			实时底盘功率和枪口热量数据，固定以 50Hz 频率发送				主控模块-->对应机器人    		常规链路
typedef __packed struct
{
	uint16_t chassis_voltage;
	uint16_t chassis_current;
	float chassis_power;
	uint16_t buffer_energy;
	uint16_t shooter_17mm_1_barrel_heat;
	uint16_t shooter_17mm_2_barrel_heat;
	uint16_t shooter_42mm_barrel_heat;
}power_heat_data_t;

//0x0203		16			机器人位置数据，固定以 1Hz 频率发送					主控模块-->对应机器人    		常规链路
typedef __packed struct
{
	float x;
	float y;
	float z;
	float angle;
}robot_pos_t;

//0x0204		6			机器人增益数据，固定以 3Hz 频率发送					服务器-->对应机器人    			常规链路
typedef __packed struct
{
	uint8_t recovery_buff;
	uint8_t cooling_buff;
	uint8_t defence_buff;
	uint8_t vulnerability_buff;
	uint16_t attack_buff;
	
}robot_buff_t;

//0x0205		2			空中支援时间数据，固定以 1Hz频率发送				服务器-->己方空中机器人			常规链路
typedef __packed struct
{
	uint8_t airforce_status;
	uint8_t time_remain;
}air_support_data_t;

//0x0206		1			伤害状态数据，伤害发生后发送						主控模块-->对应机器人				常规链路
typedef __packed struct
{
	uint8_t armor_id : 4;
	uint8_t HP_deduction_reason : 4;
}hurt_data_t;

//0x0207		7			实时射击数据，弹丸发射后发送						主控模块-->对应机器人				常规链路
typedef __packed struct
{
	uint8_t bullet_type;
	uint8_t shooter_number;
	uint8_t launching_frequency;
	float initial_speed;
}shoot_data_t;

//0x0208		6			允许发弹量，固定以 10Hz 频率发送							服务器-->己方英雄、步兵、哨兵、空中机器人			常规链路
typedef __packed struct
{
	uint16_t projectile_allowance_17mm;
	uint16_t projectile_allowance_42mm;
	uint16_t remaining_gold_coin;
}projectile_allowance_t;

//0x0209		4			机器人 RFID 模块状态，固定以3Hz 频率发送					服务器-->己方装有RFID模块的机器人			常规链路
typedef __packed struct
{
	uint32_t rfid_status;
}rfid_status_t;

//0x020A		6			飞镖选手端指令数据，固定以3Hz 频率发送			服务器-->己方飞镖机器人			常规链路
typedef __packed struct
{
	uint8_t dart_launch_opening_status;
	uint8_t reserved;
	uint16_t target_change_time;
	uint16_t latest_launch_cmd_time;
}dart_client_cmd_t;

//0x020B		40			地面机器人位置数据，固定以1Hz 频率发送			服务器-->己方哨兵机器人			常规链路
typedef __packed struct
{
	float hero_x;
	float hero_y;
	float engineer_x;
	float engineer_y;
	float standard_3_x;
	float standard_3_y;
	float standard_4_x;
	float standard_4_y;
	float standard_5_x;
	float standard_5_y;
}ground_robot_position_t;

//0x020C		6			雷达标记进度数据，固定以 1Hz频率发送				服务器-->己方雷达机器人			常规链路
typedef __packed struct
{
	uint8_t mark_hero_progress;
	uint8_t mark_engineer_progress;
	uint8_t mark_standard_3_progress;
	uint8_t mark_standard_4_progress;
	uint8_t mark_standard_5_progress;
	uint8_t mark_sentry_progress;
}radar_mark_data_t;

//0x020D		4			哨兵自主决策信息同步，固定以1Hz 频率发送			服务器→己方哨兵机器人				常规链路
typedef __packed struct
{
	uint32_t sentry_info;
}sentry_info_t;

//0x020E		1			雷达自主决策信息同步，固定以1Hz 频率发送			服务器→己方雷达机器人				常规链路
typedef __packed struct
{
	uint8_t radar_info;
}radar_info_t;

//0x0302		30			自定义控制器与机器人交互数据，发送方触发发送，频率上限为 30Hz		自定义控制器-->选手端图传连接的机器人		图传链路
typedef __packed struct
{
	uint8_t data[30];
}custom_robot_data_t;

//0x0303		15			选手端小地图交互数据，选手端触发发送		选手端点击-->服务器-->发送方旋转的己方机器人		常规链路
typedef __packed struct
{
	float target_position_x;
	float target_position_y;
	float target_position_z;
	uint8_t commd_keyboard;
	uint8_t target_robot_id;
	uint8_t cmd_source;
}map_command_t;

//0x0304		12			键鼠遥控数据，固定 30Hz 频率发送						客户端-->选手端图传连接的机器人		图传链路
typedef __packed struct
{
	uint16_t mouse_x;
	uint16_t mouse_y;
	uint16_t mouse_z;
	uint8_t left_button_down;
	uint8_t right_button_down;
	uint16_t keyboard_value;
	uint16_t reserved;
}remote_control_t;

//0x0305		10			选手端小地图接收雷达数据，频率上限为 10Hz		雷达-->服务器-->己方所有选手端		常规链路
typedef __packed struct
{
	uint16_t target_robot_id;
	float target_position_x;
	float target_position_y;
}map_robot_data_t;
//0x0306		8				自定义控制器与选手端交互数据，发送方触发发送，频率上限为 30Hz		自定义控制器-->选手端
typedef __packed struct
{
	uint16_t key_value;
	uint16_t x_position:12;
	uint16_t mouse_left:4;
	uint16_t y_position:12;
	uint16_t mouse_right:4;
	uint16_t reserved;
}custom_client_data_t;

//0x0307		103				选手端小地图接收哨兵数据，频率上限为 1Hz		哨兵/半自动控制机器人-->对应操作手选手端				常规链路
typedef __packed struct
{
	uint8_t intention;
	uint16_t start_position_x;
	uint16_t start_position_y;
	int8_t delta_x[49];
	int8_t delta_y[49];
	uint16_t sender_id;
}map_sentry_data_t;

//0x0308		34				选手端小地图接收机器人数据，频率上限为 3Hz			己方机器人-->己方选手端			常规链路
typedef __packed struct
{
uint16_t sender_id;
uint16_t receiver_id;
uint8_t user_data[30];
}custom_info_t;



extern game_status_t game_status;   
extern game_result_t game_result;
extern game_robot_HP_t game_robot_HP;
extern event_data_t event_data;
extern ext_supply_projectile_action_t ext_supply_projectile_action;
extern referee_warning_t referee_warning;
extern dart_remaining_time_t dart_remaining_time;
extern robot_status_t robot_status;
extern power_heat_data_t power_heat_data;
extern robot_pos_t robot_pos;
extern robot_buff_t robot_buff;
extern air_support_data_t air_support_data;
extern hurt_data_t hurt_data;
extern shoot_data_t shoot_data;
extern projectile_allowance_t projectile_allowance;
extern rfid_status_t rfid_status;
extern dart_client_cmd_t dart_client_cmd;
extern ground_robot_position_t ground_robot_position;
extern radar_mark_data_t radar_mark_data;
extern custom_robot_data_t custom_robot_data;
extern map_command_t map_command;
extern remote_control_t remote_control;
extern map_robot_data_t map_robot_data;
extern custom_client_data_t custom_client_data;
extern map_sentry_data_t map_sentry_data;






//裁判系统结构体数据初始化
void init_referee_struct_data(void);
/**
  * @brief          裁判系统接收的数据更新
  * @param[in]      接收的数组指针
  * @param[in]      接收的数据总长度
	* @retval         none
  */
void referee_data_updata(uint8_t *p_buff, uint16_t data_all_len);

extern void get_chassis_power_and_buffer(float *power, float *buffer);

extern uint8_t get_robot_id(void);











#pragma pack(pop)


#endif

