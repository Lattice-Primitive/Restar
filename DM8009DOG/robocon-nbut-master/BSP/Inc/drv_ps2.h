/**
 * @file drv_ps2 PS2手柄驱动
 * @author
 * @brief 基于软SPI，这是别人写的代码，能用是能用，下次优化一下吧
*/
#ifndef __DRV_PS2_H
#define __DRV_PS2_H

#ifdef __cplusplus
extern "C"
{
#endif /*__cplusplus*/

#include <stdint.h>

typedef enum
{
    PSB_SELECT,
    PSB_L3, 
    PSB_R3,
    PSB_START,
    PSB_PAD_UP,
    PSB_PAD_RIGHT,
    PSB_PAD_DOWN,
    PSB_PAD_LEFT,
    PSB_L2,
    PSB_R2,
    PSB_L1,
    PSB_R1,
    PSB_GREEN,
    PSB_RED,
    PSB_BLUE,
    PSB_PINK,

    PSB_TRIANGLE=12,
    PSB_CIRCLE,
    PSB_CROSS,
    PSB_SQUARE,
    
}PS2_Button_t;

typedef enum
{
    PSS_RX=5,
    PSS_RY,
    PSS_LX,
    PSS_LY
}PS2_Stick_t;

typedef struct 
{
    void (*CS_H)(void);
    void (*CS_L)(void);
    void (*CLK_H)(void);
    void (*CLK_L)(void);
    void (*Delay_Us)(uint32_t ticks);
    void (*DO_H)(void);
    void (*DO_L)(void);
    uint32_t (*DI_R)(void);
}PS2_Operation_t;

typedef struct
{
    uint8_t PS2_LX;
    uint8_t PS2_LY;
    uint8_t PS2_RX;
    uint8_t PS2_RY;
    uint16_t PS2_KEY;
}PS2_Data_t;

typedef struct
{
    PS2_Operation_t *opt;
    PS2_Data_t *data;
}PS2_t;

/**
 * @brief 手柄配置初始化
 * @param ps2 (PS2_t *) PS2手柄结构体
 * @return (void) NULL 
*/
extern void PS2_SetInit(PS2_t *ps2);

/**
 * @brief 读取手柄信息
 * @param ps2 (PS2_t *) PS2手柄结构体
 * @return (void) NULL 
*/
extern void PS2_Read(PS2_t *ps2);

/**
 * @brief 判断是否为红灯模式
 * @param ps2 (PS2_t *) PS2手柄结构体
 * @return (uint8_t) 红灯是否亮起 0->不亮 1->亮
*/
extern uint8_t PS2_RedLight(PS2_t *ps2);

/**
 * @brief 判断按键是否被按下
 * @param ps2 (PS2_t *) PS2手柄结构体
 * @return (uint8_t) 按键是否被按下 0->未按下 1->按下
*/
extern inline uint8_t PS2_IsButtonPress(PS2_t *ps2,PS2_Button_t button)
{
    return 1==((ps2->data->PS2_KEY)&(1<<button))?0:1;
}

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif
