#ifndef __STANDUP_H__
#define __STANDUP_H__

#include <stdint.h>

extern float Standup_LegR_Offset;
extern float Standup_LegL_Offset;
extern uint8_t flag_Standup;

void Standup_Task(void *agrument);

#endif
