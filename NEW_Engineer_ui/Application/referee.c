/*
******************************更新于2024年3月23日*********************************
参考手册：RoboMaster裁判系统串口协议附录V1.5
更新人：谭启日		2547332441@qq.com
*/

#include "referee_usart_task.h"
#include "referee.h"
#include "bsp_usart.h"
#include "CRC8_CRC16.h"
#include "string.h"

frame_header_struct_t referee_receive_header;

game_status_t game_status;   																			//比赛状态数据
game_result_t game_result;																				//比赛结果数据
game_robot_HP_t game_robot_HP;																		//机器人血量数据
event_data_t event_data;																					//场地事件数据
ext_supply_projectile_action_t ext_supply_projectile_action;			//补给站动作标识数据
referee_warning_t referee_warning;																//裁判警告数据
dart_remaining_time_t dart_remaining_time;												//飞镖发射相关数据
robot_status_t robot_status;																			//机器人性能体系数据
power_heat_data_t power_heat_data;																//实时底盘功率和枪口热量数据
robot_pos_t robot_pos;																						//机器人位置数据
robot_buff_t robot_buff;																					//机器人增益数据
air_support_data_t air_support_data;															//空中支援时间数据
hurt_data_t hurt_data;																						//伤害状态数据
shoot_data_t shoot_data;																					//实时射击数据
projectile_allowance_t projectile_allowance;											//允许发弹量
rfid_status_t rfid_status;																				//机器人 RFID 模块状态
dart_client_cmd_t dart_client_cmd;																//飞镖选手端指令数据
ground_robot_position_t ground_robot_position;										//地面机器人位置数据
radar_mark_data_t radar_mark_data;																//雷达标记进度数据
sentry_info_t sentry_info;																				//哨兵自主决策信息同步，
radar_info_t radar_info;																					//雷达自主决策信息同步
custom_robot_data_t custom_robot_data;														//自定义控制器与机器人交互数据
map_command_t map_command;																				//选手端小地图交互数据
remote_control_t remote_control;																	//键鼠遥控数据
map_robot_data_t map_robot_data;																	//选手端小地图接收雷达数据
custom_client_data_t custom_client_data;													//自定义控制器与选手端交互数据
map_sentry_data_t map_sentry_data;                                //选手端小地图接收哨兵数据
custom_info_t custom_info;                                        //选手端小地图接收机器人数据



//裁判系统结构体数据初始化
void init_referee_struct_data(void)
{
		memset(&game_status, 0, sizeof(game_status));
		memset(&game_result, 0, sizeof(game_result));
		memset(&game_robot_HP, 0, sizeof(game_robot_HP));
		memset(&event_data, 0, sizeof(event_data));
		memset(&ext_supply_projectile_action, 0, sizeof(ext_supply_projectile_action));
		memset(&referee_warning, 0, sizeof(referee_warning));
		memset(&dart_remaining_time, 0, sizeof(dart_remaining_time));
		memset(&robot_status, 0, sizeof(robot_status));
		memset(&power_heat_data, 0, sizeof(power_heat_data));
		memset(&robot_pos, 0, sizeof(robot_pos));
		memset(&robot_buff, 0, sizeof(robot_buff));
		memset(&air_support_data, 0, sizeof(air_support_data));
		memset(&hurt_data, 0, sizeof(hurt_data));
		memset(&shoot_data, 0, sizeof(shoot_data));
		memset(&projectile_allowance, 0, sizeof(projectile_allowance));
		memset(&rfid_status, 0, sizeof(rfid_status));
		memset(&dart_client_cmd, 0, sizeof(dart_client_cmd));
		memset(&ground_robot_position, 0, sizeof(ground_robot_position));
		memset(&radar_mark_data, 0, sizeof(radar_mark_data));
		memset(&sentry_info, 0, sizeof(sentry_info));
		memset(&radar_info, 0, sizeof(radar_info));
		memset(&custom_robot_data, 0, sizeof(custom_robot_data));
		memset(&map_command, 0, sizeof(map_command));
		memset(&remote_control, 0, sizeof(remote_control));
		memset(&map_robot_data, 0, sizeof(map_robot_data));
		memset(&custom_client_data, 0, sizeof(custom_client_data));
		memset(&map_sentry_data, 0, sizeof(map_sentry_data));
		memset(&custom_info, 0, sizeof(custom_info));
}



/**
  * @brief          裁判系统接收的数据更新
  * @param[in]      接收的数组指针
  * @param[in]      接收的数据总长度
	* @retval         none
  */
void referee_data_updata(uint8_t *p_buff, uint16_t data_all_len){
	static uint16_t len = 0;			
	for( ; data_all_len > REFEREE_DATA_MIN_SIZE ; ){			//总数据长度大于接收数据最小尺寸时，后续还有数据
		memcpy(&referee_receive_header, p_buff, REF_PROTOCOL_HEADER_SIZE);				//数据帧头传递
		if(verify_CRC8_check_sum((unsigned char*)&referee_receive_header,REF_PROTOCOL_HEADER_SIZE)){  //CRC8校验
			len = REF_PROTOCOL_HEADER_SIZE+REF_PROTOCOL_CMD_SIZE+REF_PROTOCOL_CRC16_SIZE+referee_receive_header.data_length;
			if(verify_CRC16_check_sum(p_buff,len)){    //CRC16校验
				static uint16_t CMD_ID = 0;
				memcpy(&CMD_ID,p_buff+REF_PROTOCOL_HEADER_SIZE,REF_PROTOCOL_CMD_SIZE);			//命令码传递
				switch(CMD_ID){				//根据ID传递数据
					case GAME_STATUS_CMD_ID :
						memcpy(&game_status,p_buff+REF_PROTOCOL_HEADER_SIZE+REF_PROTOCOL_CMD_SIZE,referee_receive_header.data_length);
						break;
					case GAME_RESULT_CMD_ID :
						memcpy(&game_result,p_buff+REF_PROTOCOL_HEADER_SIZE+REF_PROTOCOL_CMD_SIZE,referee_receive_header.data_length);
						break;
					case GAME_ROBOT_HP_CMD_ID :
						memcpy(&game_robot_HP,p_buff+REF_PROTOCOL_HEADER_SIZE+REF_PROTOCOL_CMD_SIZE,referee_receive_header.data_length);
						break;
					case EVENTS_DATA_CMD_ID :
						memcpy(&event_data,p_buff+REF_PROTOCOL_HEADER_SIZE+REF_PROTOCOL_CMD_SIZE,referee_receive_header.data_length);
						break;
					case EXT_SUPPLY_PROJECTILE_ACTION_CMD_ID :
						memcpy(&ext_supply_projectile_action,p_buff+REF_PROTOCOL_HEADER_SIZE+REF_PROTOCOL_CMD_SIZE,referee_receive_header.data_length);
						break;
					case REFEREE_WARNING_CMD_ID :
						memcpy(&referee_warning,p_buff+REF_PROTOCOL_HEADER_SIZE+REF_PROTOCOL_CMD_SIZE,referee_receive_header.data_length);
						break;
					case DART_REMAINING_TIME_CMD_ID :
						memcpy(&dart_remaining_time,p_buff+REF_PROTOCOL_HEADER_SIZE+REF_PROTOCOL_CMD_SIZE,referee_receive_header.data_length);
						break;
					case ROBOT_STATUS_CMD_ID :
						memcpy(&robot_status,p_buff+REF_PROTOCOL_HEADER_SIZE+REF_PROTOCOL_CMD_SIZE,referee_receive_header.data_length);
						break;
					case POWER_HEAT_DATA_CMD_ID :
						memcpy(&power_heat_data,p_buff+REF_PROTOCOL_HEADER_SIZE+REF_PROTOCOL_CMD_SIZE,referee_receive_header.data_length);
						break;
					case ROBOT_POS_CMD_ID :
						memcpy(&robot_pos,p_buff+REF_PROTOCOL_HEADER_SIZE+REF_PROTOCOL_CMD_SIZE,referee_receive_header.data_length);
						break;
					case ROBOT_BUFF_CMD_ID :
						memcpy(&robot_buff,p_buff+REF_PROTOCOL_HEADER_SIZE+REF_PROTOCOL_CMD_SIZE,referee_receive_header.data_length);
						break;
					case AIR_SUPPORT_DATA_CMD_ID :
						memcpy(&air_support_data,p_buff+REF_PROTOCOL_HEADER_SIZE+REF_PROTOCOL_CMD_SIZE,referee_receive_header.data_length);
						break;
					case HURT_DATA_CMD_ID :
						memcpy(&hurt_data,p_buff+REF_PROTOCOL_HEADER_SIZE+REF_PROTOCOL_CMD_SIZE,referee_receive_header.data_length);
						break;
					case SHOOT_DATA_CMD_ID :
						memcpy(&shoot_data,p_buff+REF_PROTOCOL_HEADER_SIZE+REF_PROTOCOL_CMD_SIZE,referee_receive_header.data_length);
						break;
					case PROJECTILE_ALLOWANCE_CMD_ID :
						memcpy(&projectile_allowance,p_buff+REF_PROTOCOL_HEADER_SIZE+REF_PROTOCOL_CMD_SIZE,referee_receive_header.data_length);
						break;
					case RFID_STATUS_CMD_ID :
						memcpy(&rfid_status,p_buff+REF_PROTOCOL_HEADER_SIZE+REF_PROTOCOL_CMD_SIZE,referee_receive_header.data_length);
						break;
					case DART_CLIENT_CMD_ID :
						memcpy(&dart_client_cmd,p_buff+REF_PROTOCOL_HEADER_SIZE+REF_PROTOCOL_CMD_SIZE,referee_receive_header.data_length);
						break;
					case GROUND_ROBOT_POSITION_CMD_ID :
						memcpy(&ground_robot_position,p_buff+REF_PROTOCOL_HEADER_SIZE+REF_PROTOCOL_CMD_SIZE,referee_receive_header.data_length);
						break;
					case RADAR_MARK_DATA_CMD_ID :
						memcpy(&radar_mark_data,p_buff+REF_PROTOCOL_HEADER_SIZE+REF_PROTOCOL_CMD_SIZE,referee_receive_header.data_length);
						break;
					case SENTRY_INFO_CMD_ID :
						memcpy(&sentry_info,p_buff+REF_PROTOCOL_HEADER_SIZE+REF_PROTOCOL_CMD_SIZE,referee_receive_header.data_length);
						break;
					case RADAR_INFO_CMD_ID :
						memcpy(&radar_info,p_buff+REF_PROTOCOL_HEADER_SIZE+REF_PROTOCOL_CMD_SIZE,referee_receive_header.data_length);
						break;
					case CUSTOM_ROBOT_DATA_CMD_ID	:
						memcpy(&custom_robot_data,p_buff+REF_PROTOCOL_HEADER_SIZE+REF_PROTOCOL_CMD_SIZE,referee_receive_header.data_length);
						break;
					case MAP_COMMAND_CMD_ID :
						memcpy(&map_command,p_buff+REF_PROTOCOL_HEADER_SIZE+REF_PROTOCOL_CMD_SIZE,referee_receive_header.data_length);
						break;
					case REMOTE_CONTROL_CMD_ID :
						memcpy(&game_result,p_buff+REF_PROTOCOL_HEADER_SIZE+REF_PROTOCOL_CMD_SIZE,referee_receive_header.data_length);
						break;
					case MAP_ROBOT_DATA_CMD_ID :
						memcpy(&map_robot_data,p_buff+REF_PROTOCOL_HEADER_SIZE+REF_PROTOCOL_CMD_SIZE,referee_receive_header.data_length);
						break;
					case CUSTOM_CLIENT_CATA_CMD_ID :
						memcpy(&custom_client_data,p_buff+REF_PROTOCOL_HEADER_SIZE+REF_PROTOCOL_CMD_SIZE,referee_receive_header.data_length);
						break;
					case MAP_SENTRY_DATA_CMD_ID :
						memcpy(&map_sentry_data,p_buff+REF_PROTOCOL_HEADER_SIZE+REF_PROTOCOL_CMD_SIZE,referee_receive_header.data_length);
						break;
					case CUSTOM_INFO_CMD_ID :
						memcpy(&custom_info,p_buff+REF_PROTOCOL_HEADER_SIZE+REF_PROTOCOL_CMD_SIZE,referee_receive_header.data_length);
						break;
				}
				
				data_all_len = data_all_len > len ? (data_all_len - len) : 0;   //剩余数据总长度
				p_buff += len;		//移除上一段数据		
			}
			else{				//除去数据段
				data_all_len = data_all_len > len ? (data_all_len - len) : 0;   //剩余数据总长度
				p_buff += len;		//移除上一段数据		
			}
		}
		else{   
			return;
		}
	}
}




void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer)
{
    *power = power_heat_data.chassis_power;
    *buffer = power_heat_data.buffer_energy;
}


uint8_t get_robot_id(void)
{
    return robot_status.robot_id;
}





