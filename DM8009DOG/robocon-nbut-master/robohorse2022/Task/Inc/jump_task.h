#ifndef __JUMP_TASK_H__
#define __JUMP_TASK_H__
#include "stanford_type.h"
void Jump_Task(void *argument);

/**
 * @brief 将目标极坐标转化成电机角度并输出到电机
 * @param Action_Polar_Buffer 目标极坐标 数组长度必须为4
 * @return void
 */
void Polar_setCoord(Polar_Coord_Data_t *Action_Polar_Buffer);
#endif
