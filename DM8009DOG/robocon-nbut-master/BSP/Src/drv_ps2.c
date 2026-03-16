#include "drv_ps2.h"

static uint8_t Data[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //数据存储数组

void PS2_Cmd(PS2_Operation_t* opt, uint8_t cmd)
{
	volatile uint16_t ref=0x01;
	Data[1] = 0;
	for(ref=0x01;ref<0x0100;ref<<=1)
	{
		if(ref&cmd)
		{
			opt->DO_H();                   //输出一位控制位
		}
		else opt->DO_L();

		opt->CLK_H();                        //时钟拉高
		opt->Delay_Us(5);
		opt->CLK_L();
		opt->Delay_Us(5);
		opt->CLK_H();
		if(opt->DI_R())
			Data[1] = ref|Data[1];
	}
	opt->Delay_Us(16);
}

void PS2_ReadData(PS2_Operation_t *opt)
{
	volatile uint8_t byte=0;
	volatile uint16_t ref=0x01;
	opt->CS_L();
	PS2_Cmd(opt,0x01);  //开始命令
	PS2_Cmd(opt,0x42);  //请求数据
	for(byte=2;byte<9;byte++)          //开始接受数据
	{
		for(ref=0x01;ref<0x100;ref<<=1)
		{
			opt->CLK_H();
			opt->Delay_Us(5);
			opt->CLK_L();
			opt->Delay_Us(5);
			opt->CLK_H();
		      if(opt->DI_R())
		      	Data[byte] = ref|Data[byte];
		}
        opt->Delay_Us(16);
	}
	opt->CS_H();
}

uint8_t PS2_RedLight(PS2_t *ps2)
{
    ps2->opt->CS_L();
	PS2_Cmd(ps2->opt,0x01);  //开始命令
	PS2_Cmd(ps2->opt,0x42);  //请求数据
	ps2->opt->CS_H();
	if( Data[1] == 0X73)   return 0 ;
	else return 1;
}

//清除数据缓冲区
void PS2_ClearData(void)
{
	uint8_t a;
	for(a=0;a<9;a++)
		Data[a]=0x00;
}

//得到一个摇杆的模拟量	 范围0~256
uint8_t PS2_AnologData(PS2_Stick_t stick)
{
	return Data[stick];
}

//对读出来的PS2的数据进行处理,只处理按键部分  
//只有一个按键按下时按下为0， 未按下为1
uint16_t PS2_DataKey(PS2_Operation_t *opt)
{
    uint16_t Handkey;

	PS2_ClearData();
	PS2_ReadData(opt);

	Handkey=(Data[4]<<8)|Data[3];     //这是16个按键  按下为0， 未按下为1
	/* for(index=0;index<16;index++)
	{	    
		if(Handkey&(1<<index)==0)
		return index;
	} */
	return Handkey;
}

void PS2_Vibration(PS2_Operation_t *opt,uint8_t motor1, uint8_t motor2)
{
	opt->CS_L();
	opt->Delay_Us(16);
    PS2_Cmd(opt,0x01);  //开始命令
	PS2_Cmd(opt,0x42);  //请求数据
	PS2_Cmd(opt,0X00);
	PS2_Cmd(opt,motor1);
	PS2_Cmd(opt,motor2);
	PS2_Cmd(opt,0X00);
	PS2_Cmd(opt,0X00);
	PS2_Cmd(opt,0X00);
	PS2_Cmd(opt,0X00);
	opt->CS_H();
	opt->Delay_Us(16);  
}

//short poll
void PS2_ShortPoll(PS2_Operation_t *opt)
{
    opt->CS_L();
	opt->Delay_Us(16);
	PS2_Cmd(opt,0x01);  
	PS2_Cmd(opt,0x42);  
	PS2_Cmd(opt,0X00);
	PS2_Cmd(opt,0x00);
	PS2_Cmd(opt,0x00);
	opt->CS_H();
	opt->Delay_Us(16);
}

//进入配置
void PS2_EnterConfing(PS2_Operation_t *opt)
{
    opt->CS_L();
	opt->Delay_Us(16);
	PS2_Cmd(opt,0x01);  
	PS2_Cmd(opt,0x43);  
	PS2_Cmd(opt,0X00);
	PS2_Cmd(opt,0x01);
	PS2_Cmd(opt,0x00);
	PS2_Cmd(opt,0X00);
	PS2_Cmd(opt,0X00);
	PS2_Cmd(opt,0X00);
	PS2_Cmd(opt,0X00);
	opt->CS_H();
	opt->Delay_Us(16);
}

//发送模式设置
void PS2_TurnOnAnalogMode(PS2_Operation_t *opt)
{
	opt->CS_L();
	PS2_Cmd(opt,0x01);  
	PS2_Cmd(opt,0x44);  
	PS2_Cmd(opt,0X00);
	PS2_Cmd(opt,0x01); //analog=0x01;digital=0x00  软件设置发送模式
	PS2_Cmd(opt,0xee); //Ox03锁存设置，即不可通过按键“MODE”设置模式。
				   //0xEE不锁存软件设置，可通过按键“MODE”设置模式。
	PS2_Cmd(opt,0X00);
	PS2_Cmd(opt,0X00);
	PS2_Cmd(opt,0X00);
	PS2_Cmd(opt,0X00);
	opt->CS_H();
	opt->Delay_Us(16);
}

//振动设置
void PS2_VibrationMode(PS2_Operation_t *opt)
{
	opt->CS_L();
	opt->Delay_Us(16);
	PS2_Cmd(opt,0x01);  
	PS2_Cmd(opt,0x4D);  
	PS2_Cmd(opt,0X00);
	PS2_Cmd(opt,0x00);
	PS2_Cmd(opt,0X01);
	opt->CS_H();
	opt->Delay_Us(16);	
}

//完成并保存配置
void PS2_ExitConfing(PS2_Operation_t *opt)
{
    opt->CS_L();
	opt->Delay_Us(16);
	PS2_Cmd(opt,0x01);  
	PS2_Cmd(opt,0x43);  
	PS2_Cmd(opt,0X00);
	PS2_Cmd(opt,0x00);
	PS2_Cmd(opt,0x5A);
	PS2_Cmd(opt,0x5A);
	PS2_Cmd(opt,0x5A);
	PS2_Cmd(opt,0x5A);
	PS2_Cmd(opt,0x5A);
	opt->CS_H();
	opt->Delay_Us(16);
}

//手柄配置初始化
void PS2_SetInit(PS2_t *ps2)
{
	PS2_ShortPoll(ps2->opt);
	PS2_ShortPoll(ps2->opt);
	PS2_ShortPoll(ps2->opt);
	PS2_EnterConfing(ps2->opt);		//进入配置模式
	PS2_TurnOnAnalogMode(ps2->opt);	//“红绿灯”配置模式，并选择是否保存
	//PS2_VibrationMode();	//开启震动模式
	PS2_ExitConfing(ps2->opt);		//完成并保存配置	
}

//读取手柄信息
void PS2_Read(PS2_t *ps2)
{
	ps2->data->PS2_KEY=PS2_DataKey(ps2->opt);
	ps2->data->PS2_LX=PS2_AnologData(PSS_LX);
	ps2->data->PS2_LY=PS2_AnologData(PSS_LY);
	ps2->data->PS2_RX=PS2_AnologData(PSS_RX);
	ps2->data->PS2_RY=PS2_AnologData(PSS_RY);
}
