#include "app_serial_std.h"
#pragma import(__use_no_semihosting) //无半主机

static Serial_Std_operation_t *Now_Serial_Std;

void Serial_Std_Init(Serial_Std_operation_t* opt)
{
    Now_Serial_Std=opt;//将该结构体中的函数用于printf重定义
}
 
struct __FILE
{
    int handle;
};
 
FILE __stdout;
FILE __stdin;
 
int fputc(int ch, FILE *f)
{
    return Now_Serial_Std->Serial_Std_Putc((uint8_t)ch); // 发送字符
}

int fgetc(FILE *f)
{
    return (Now_Serial_Std->Serial_Std_Putc(Now_Serial_Std->Serial_Std_Getc())); //将接收到的字符发送回显
    /*调用scanf()在串口中输入数据时，必须以空格结束，否则无法完成发送*/
}

void _ttywrch(int ch)
{
    Now_Serial_Std->Serial_Std_Putc((uint8_t)ch);
}

int _ferror(FILE *f) 
{
  /* Your implementation of ferror */
  return EOF;
}

void _sys_exit(int return_code)
{
    label:goto label;
}
