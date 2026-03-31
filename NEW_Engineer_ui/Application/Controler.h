#ifndef CONTROLER_H
#define CONTROLER_H

#include "enginer_task.h"

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

void con_init(void);
		
void ALL_control(float set_angle);





#endif 
