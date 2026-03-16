/**
 * @file drv_oled.h 驱动层 OLED
 * @author Zhiyuan Mao (2019th) (全平台适用)
 * @note 用于OLED显示屏
*/
#ifndef __DRV_OLED_H
#define __DRV_OLED_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

/**
 * 字符集(蹲一个中文字库，虽然用处不大)
 * */
#define ASCII_1206 0x00//12*12 ASCII字符集点阵
#define ASCII_1608 0x01//16*16 ASCII字符集点阵
#define ASCII_2412 0x02//24*24 ASICII字符集点阵

typedef struct 
{
    uint8_t (*OLED_Send_DataByte)(uint8_t data);
    uint8_t (*OLED_Send_CommandByte)(uint8_t cmd);
}OLED_Operate_t;

typedef struct
{
    uint32_t gram_buffer; //图形存储缓存指针,需要把指针转化成uint32_t
    uint32_t row; //OLED显示屏横排像素数
    uint32_t column; //OLED显示屏纵列像素数
    OLED_Operate_t* opt; //OLED操作结构体
    uint8_t addr; //器件i2c地址
}OLED_t;

/**
 * @brief 初始化oled屏
 * @param oled (OLED_t*)OLED结构体指针
 * @return (void) NULL
*/
extern void OLED_Init(OLED_t *oled);

/**
 * @brief 开启oled屏显示
 * @param oled (OLED_t*)OLED结构体指针
 * @return NULL
*/
extern void OLED_Display_On(OLED_t *oled);

/**
 * @brief 关闭oled屏显示
 * @param oled (OLED_t*)OLED结构体指针
 * @return NULL
*/
extern void OLED_Display_Off(OLED_t *oled);

/**
 * @brief 刷新oled显示
 * @param oled (OLED_t*)OLED结构体指针
 * @return NULL
*/
extern void OLED_Gram_refresh(OLED_t *oled);

/**
 * @brief 刷新oled显示
 * @param oled (OLED_t*)OLED结构体指针
 * @return NULL
*/
extern void OLED_Gram_Clear(OLED_t *oled);

/**
 * @brief oled点绘制
 * @param oled (OLED_t*)OLED结构体指针
 * @param x (uint32_t)横坐标位置
 * @param y (uint32_t)纵坐标位置
 * @param isfilled (uint8_t)是否填充
 * @arg 1:填充
 * @arg 0:清空
 * @return NULL
*/
extern void OLED_Gram_DrawPoint(OLED_t *oled, uint32_t x, uint32_t y, uint8_t isfilled);

/**
 * @brief oled矩形绘制
 * @param oled (OLED_t*)OLED结构体指针
 * @param x1 (uint32_t)第一个顶点横坐标位置
 * @param y1 (uint32_t)第一个顶点纵坐标位置
 * @param x2 (uint32_t)第二个顶点横坐标位置
 * @param y2 (uint32_t)第二个顶点纵坐标位置
 * @param isfilled (uint8_t)是否填充
 * @arg 1:填充
 * @arg 0:清空
 * @return NULL
*/
extern void OLED_Gram_DrawRectangle(OLED_t *oled, uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2, uint8_t isfilled);

/**
 * @brief oled字符绘制
 * @param oled (OLED_t*)OLED结构体指针
 * @param x (uint32_t)顶点(字符左上角)横坐标位置
 * @param y (uint32_t)顶点(字符左上角)纵坐标位置
 * @param chr (uint8_t) 要显示的字符
 * @param font_size (uint8_t)字体大小
 * @param isfilled (uint8_t)是否填充
*/
extern void OLED_Gram_DrawChar(OLED_t *oled, uint32_t x, uint32_t y, uint8_t chr, uint8_t font, uint8_t isfilled);

/**
 * @brief oled数字绘制
 * @param oled (OLED_t*)OLED结构体指针
 * @param x (uint32_t)顶点(字符左上角)横坐标位置
 * @param y (uint32_t)顶点(字符左上角)纵坐标位置
 * @param num (int32_t) 要显示的数字
 * @param font_size (uint8_t)字体大小
 * @param isfilled (uint8_t)是否填充
*/
extern void OLED_Gram_DrawNum(OLED_t *oled, uint32_t x, uint32_t y, int32_t num, uint8_t font, uint8_t isfilled);

/**
 * @brief oled字符串绘制,可以考虑与sprintf()配合
 * @param oled (OLED_t*)OLED结构体指针
 * @param x (uint32_t)顶点(字符左上角)横坐标位置
 * @param y (uint32_t)顶点(字符左上角)纵坐标位置
 * @param pstr (uint8_t)字符串指针
 * @param str_size (uint32_t)字符串长度
 * @param font_size (uint8_t)字体大小
 * @param isfilled (uint8_t)是否填充
*/
extern void OLED_Gram_DrawString(OLED_t *oled, uint32_t x, uint32_t y, uint8_t *pstr, uint32_t str_size, uint8_t font, uint8_t isfilled);

#ifdef __cplusplus
}
#endif

#endif // !__DRV_OLED_H
