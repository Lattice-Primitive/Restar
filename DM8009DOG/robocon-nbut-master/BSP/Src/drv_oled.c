#include "drv_oled.h"
#include "oled_font.h"

extern void OLED_Init(OLED_t *oled)
{
    oled->opt->OLED_Send_CommandByte(0xAE); //关闭显示
    oled->opt->OLED_Send_CommandByte(0xD5); //设置时钟分频因子,震荡频率
    oled->opt->OLED_Send_CommandByte(0x80); //[3:0],分频因子;[7:4],震荡频率
    oled->opt->OLED_Send_CommandByte(0xA8); //设置驱动路数
    oled->opt->OLED_Send_CommandByte(0x3F); //默认0X3F(1/64)
    oled->opt->OLED_Send_CommandByte(0xD3); //设置显示偏移
    oled->opt->OLED_Send_CommandByte(0x00); //默认为0

    oled->opt->OLED_Send_CommandByte(0x40); //设置显示开始行 [5:0],行数.

    oled->opt->OLED_Send_CommandByte(0x8D);//电荷泵设置
    oled->opt->OLED_Send_CommandByte(0x14);//bit2，开启/关闭
    oled->opt->OLED_Send_CommandByte(0x20);//设置内存地址模式
    oled->opt->OLED_Send_CommandByte(0x02);//[1:0],00，列地址模式;01，行地址模式;10,页地址模式;默认10;
    oled->opt->OLED_Send_CommandByte(0xA1);//段重定义设置,bit0:0,0->0;1,0->127;
    oled->opt->OLED_Send_CommandByte(0xC0);//设置COM扫描方向;bit3:0,普通模式;1,重定义模式 COM[N-1]->COM0;N:驱动路数
    oled->opt->OLED_Send_CommandByte(0xDA);//设置COM硬件引脚配置
    oled->opt->OLED_Send_CommandByte(0x12);//[5:4]配置

    oled->opt->OLED_Send_CommandByte(0x81);//对比度设置
    oled->opt->OLED_Send_CommandByte(0xEF);//1~255;默认0X7F (亮度设置,越大越亮)
    oled->opt->OLED_Send_CommandByte(0xD9);//设置预充电周期
    oled->opt->OLED_Send_CommandByte(0xf1);//[3:0],PHASE 1;[7:4],PHASE 2;
    oled->opt->OLED_Send_CommandByte(0xDB);//设置VCOMH 电压倍率
    oled->opt->OLED_Send_CommandByte(0x30);//[6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;

    oled->opt->OLED_Send_CommandByte(0xA4);//全局显示开启;bit0:1,开启;0,关闭;(白屏/黑屏)
    oled->opt->OLED_Send_CommandByte(0xA6);//设置显示方式;bit0:1,反相显示;0,正常显示
    oled->opt->OLED_Send_CommandByte(0xAF);//开启显示
    OLED_Gram_Clear(oled);
}

void OLED_Display_On(OLED_t *oled)
{
    oled->opt->OLED_Send_CommandByte(0X8D); //SET DCDC命令
    oled->opt->OLED_Send_CommandByte(0X14); //DCDC ON
    oled->opt->OLED_Send_CommandByte(0XAF); //DISPLAY ON
}

void OLED_Display_Off(OLED_t *oled)
{
    oled->opt->OLED_Send_CommandByte(0X8D); //SET DCDC命令
    oled->opt->OLED_Send_CommandByte(0X10); //DCDC ON
    oled->opt->OLED_Send_CommandByte(0XAE); //DISPLAY ON
}

void OLED_Gram_refresh(OLED_t *oled)
{
    uint8_t i, n;
    for (i = 0; i < 8; i++)
    {
        oled->opt->OLED_Send_CommandByte((0xb0 | i)); //设置页地址（0~7）
        oled->opt->OLED_Send_CommandByte(0x00);       //设置显示位置—列低地址
        oled->opt->OLED_Send_CommandByte(0x10);       //设置显示位置—列高地址
        for (n = 0; n < 128; n++)
            oled->opt->OLED_Send_DataByte(*((uint8_t*)(oled->gram_buffer+n*oled->row/8+i)));
    }
}

void OLED_Gram_Clear(OLED_t *oled)
{
    uint8_t i, n;
    for (i = 0; i < 8; i++)
        for (n = 0; n < 128; n++)
            *((uint8_t*)(oled->gram_buffer+n*oled->row/8+i)) = 0X00;
}

void OLED_Gram_DrawPoint(OLED_t *oled, uint32_t x, uint32_t y, uint8_t isfilled)
{
    uint32_t pos, bx, temp = 0;
    if (x > oled->row || y > oled->column)
        return; //超出范围了.
    pos = 7 - y / 8;
    bx = y % 8;
    temp = 1 << (7 - bx);
    if (isfilled)
        *((uint8_t*)(oled->gram_buffer+x*oled->row/8+pos)) |= temp;
    else
        *((uint8_t*)(oled->gram_buffer+x*oled->row/8+pos)) &= ~temp;
}

void OLED_Gram_DrawRectangle(OLED_t *oled, uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2, uint8_t isfilled)
{
    uint32_t x, y, t;
    if (x1 > x2)
        t = x1;
    x1 = x2, x2 = t;
    if (y1 > y2)
        t = y1;
    y1 = y2, y2 = t;
    for (x = x1; x <= x2; x++)
    {
        for (y = y1; y <= y2; y++)
            OLED_Gram_DrawPoint(oled, x, y, isfilled);
    }
}

void OLED_Gram_DrawChar(OLED_t *oled, uint32_t x, uint32_t y, uint8_t chr, uint8_t font, uint8_t isfilled)
{
    uint8_t temp, t, t1, chr_size, font_size;
    uint8_t y0 = y;
    uint32_t chr_font;
    switch (font)
    {
    case ASCII_1206:
        chr_size = 12;
        font_size = 12;
        chr_font = (uint32_t)asc2_1206;
        break;

    case ASCII_1608:
        chr_size = 16;
        font_size = 16;
        chr_font = (uint32_t)asc2_1608;
        break;

    case ASCII_2412:
        chr_size = 36;
        font_size = 24;
        chr_font = (uint32_t)asc2_2412;
        break;

    default:
        return;
    }
    chr = chr - ' ';
    for (t = 0; t < chr_size; t++)
    {
        temp = *((uint8_t*)(chr_font+chr*chr_size+t));
        for (t1 = 0; t1 < 8; t1++)
        {
            if (temp & 0x80)
                OLED_Gram_DrawPoint(oled, x, y, isfilled);
            else
                OLED_Gram_DrawPoint(oled, x, y, !isfilled);
            temp <<= 1;
            y++;
            if ((y - y0) == font_size)
            {
                y = y0;
                x++;
                break;
            }
        }
    }
}

void OLED_Gram_DrawNum(OLED_t *oled, uint32_t x, uint32_t y, int32_t num, uint8_t font, uint8_t isfilled)
{
    int8_t font_size;
    switch (font)
    {
    case ASCII_1206:
        font_size = 12;
        break;

    case ASCII_1608:
        font_size = 16;
        break;

    case ASCII_2412:
        font_size = 24;
        break;

    default:
        return;
    }
    int32_t decs = 1000000000;
    if (num < 0)
    {
        OLED_Gram_DrawChar(oled, x, y, '-', font, isfilled);
        num = -num;
        x += font_size / 2;
    }
    else if (num == 0)
    {
        OLED_Gram_DrawChar(oled,x, y,'0', font, isfilled);
        x += font_size / 2;
    }
    while (decs > num)
        decs /= 10;
    while (decs > 0)
    {
        if (decs <= num)
        {
            OLED_Gram_DrawChar(oled, x, y, ('0' + num / decs), font, isfilled);
            num %= decs;
        }
        else
        {
            OLED_Gram_DrawChar(oled, x, y, '0', font, isfilled);
        }
        x += font_size / 2;
        decs /= 10;
        if (x > (128 - (font_size / 2)))
        {
            x = 0;
            y += font_size;
        }
        if (y > (64 - font_size))
        {
            y = x = 0;
            OLED_Gram_Clear(oled);
        }
    }
}

void OLED_Gram_DrawString(OLED_t *oled, uint32_t x, uint32_t y, uint8_t *pstr, uint32_t str_size, uint8_t font, uint8_t isfilled)
{
    uint8_t font_size;
    switch (font)
    {
    case ASCII_1206:
        font_size = 12;
        break;

    case ASCII_1608:
        font_size = 16;
        break;

    case ASCII_2412:
        font_size = 24;
        break;

    default:
        return;
    }
    for (uint32_t i = 0; i < str_size; i++)
    {
        if (x > (128 - (font_size / 2)))
        {
            x = 0;
            y += font_size;
        }
        if (y > (64 - font_size))
        {
            y = x = 0;
            OLED_Gram_Clear(oled);
        }
        OLED_Gram_DrawChar(oled, x, y, pstr[i], font, isfilled);
        x += font_size / 2;
    }
}
