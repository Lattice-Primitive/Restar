/*************************************************************

RM自定义UI协议       基于RM2020学生串口通信协议V1.1


**************************************************************/


#include "RM_Cilent_UI.h"
#include "string.h"
#include "referee.h"
#include "keyboard.h"
#include "CAN_receive.h"
#include "enginer_task.h"


unsigned char UI_Seq;                 //包序号
uint16_t Robot_ID,Cilent_ID;					//机器人ID和队员选手端ID，在UI初始化时对裁判系统接收数据进行更新此ID
uint8_t usart6_tx_buf[256];						//发送数组
uint16_t usart6_tx_len = 0;						//发送数据长度
uint8_t UI_init_flag = 1;							//UI初始化标志
uint8_t show_num = 0;									//显示到哪个位置的变量
uint8_t loop_end_flag = 0;						//循环结束标志

int x_distance, z_distance;

uint16_t 	UI_time;											//更新时间，用于进行任务更新作用
uint8_t		tim_40ms_flag = 0;


UI_Data G1,G2,G3,G4,G5,G6,G7,G8,G9,G10,G11,G12,G13,G14,G15,G16,G17,G18,G19,G20,G21,G22,G23,G24,G25,G26;   //图形
UI_Data F1,F2,F3,F4,F5,F6,F7;           																					//浮点数
UI_Data I1,I2,I3,I4,I5,I6,I7;																											//整型
String_Data S1,S2,S3,S4,S5,S6,S7,S8,S9,S10,S11,S12,S13,S14;    										//字符


/************************UI更新任务********************
放在裁判系统任务里面，周期10ms

每个显示段内最多显示(7个图形数据：图形、浮点型、整型)或(1个字符串)
即：每个显示段只能用一次UI_ReFresh()或Char_ReFresh()


动态UI其实就是每个显示段根据不同的变量修改UI显示的颜色或内容

******************************************************/
void UI_updata_task(void)
{
//		keyboard_ctrl.UI_open_flag = 1;		//调试用
		if (keyboard_ctrl.UI_open_flag == 1){
				UI_time++;
			
				if (UI_time >= 4)//40ms标志
				{			
						UI_time = 0;
						tim_40ms_flag = 1;
				}
			
				if (tim_40ms_flag == 1){			//40ms进行更新一次
						tim_40ms_flag = 0;
//UI初始化显示内容********************************************************************************************************
						if (UI_init_flag == 1)
						{			
								if (show_num == 0)//第1显示段**********************************************
								{			
										loop_end_flag = 0;//循环结束标志
						
										//根据裁判系统的机器人信息更新UI发送ID
										Robot_ID = robot_status.robot_id;
										Cilent_ID = robot_status.robot_id + 0x100;
										//先清除所有图层信息
										UI_Delete(UI_Data_Del_ALL,0);			
										//清空配置信息
//										memset(&G1,0,sizeof(G1));
//										memset(&G2,0,sizeof(G2));
//										memset(&G3,0,sizeof(G3));
//										memset(&G4,0,sizeof(G4));
//										memset(&G5,0,sizeof(G5));
//									
//										memset(&S1,0,sizeof(S1));
//										memset(&S2,0,sizeof(S2));
//										memset(&F1,0,sizeof(F1));
//										memset(&I1,0,sizeof(I1));
								}
								if (show_num == 1)//第2显示段***********************************************
								{
										loop_end_flag = 0;//循环结束标志
										
										

										//将图形内容推送至缓存数组内
										UI_ReFresh(3,G1,G2,G3);
								}
								if (show_num == 2)//第3显示段***********************************************
								{			
										loop_end_flag = 0;//循环结束标志
					
										Circle_Draw(&G6,"G6",UI_Graph_ADD,9,UI_Color_White,10,1640,825,9);					//判断传送带是否开启							动态
										Line_Draw(&G7,"G7",UI_Graph_ADD,9,UI_Color_Purplish_red,20,710,820,1210,820);					//X轴显示进度条	动态
										Line_Draw(&G8,"G8",UI_Graph_ADD,9,UI_Color_Purplish_red,20,1650,750,1650,350);					//Z抬升显示进度条	动态
										Circle_Draw(&G9,"G9",UI_Graph_ADD,9,UI_Color_Orange,5,957,538,80);					//判断气泵是否开启						动态
										Circle_Draw(&G10,"G10",UI_Graph_ADD,9,UI_Color_Green,10,1640,775,9);				//判断按键是否开启							动态
										
										//将图形内容推送至缓存数组内
										UI_ReFresh(7,G4,G5,G6,G7,G8,G9,G10);
								}
								if (show_num == 3)//第4显示段***********************************************
								{			
										loop_end_flag = 0;//循环结束标志
									
										Arc_Draw(&G11,"G11",UI_Graph_ADD,9,UI_Color_White, 8,	80, 800, 180, 0, 40, 40);						//不是吧哥们							静态
										Line_Draw(&G12, "G12", UI_Graph_ADD, 9, UI_Color_White, 8, 80, 840, 160, 840);
										Circle_Draw(&G13, "G13", UI_Graph_ADD, 9, UI_Color_White, 8, 140, 850, 4);
										Circle_Draw(&G14, "G14", UI_Graph_ADD, 9, UI_Color_White, 8, 125, 830, 4);
										
										//将图形内容推送至缓存数组内
										UI_ReFresh(7,G11,G12,G13,G14,G15,G16,G17);
								}
								if (show_num == 4)//第5显示段***********************************************
								{			
										loop_end_flag = 0;//循环结束标志
										
										//X进度条描边
										Line_Draw(&G15, "G15", UI_Graph_ADD, 8, UI_Color_White, 4, 708, 832, 708, 808);
										Line_Draw(&G16, "G16", UI_Graph_ADD, 8, UI_Color_White, 4, 1212, 832, 1212, 808);
										Line_Draw(&G17, "G17", UI_Graph_ADD, 8, UI_Color_White, 4, 708, 832, 1212, 832);
										Line_Draw(&G18, "G18", UI_Graph_ADD, 8, UI_Color_White, 4, 708, 808, 1212, 808);
										//将图形内容推送至缓存数组内
										UI_ReFresh(5,G15,G16,G17,G18);
								}
								if (show_num == 5)//第6显示段***********************************************
								{			
										loop_end_flag = 0;//循环结束标志
						
										Char_Draw(&S3,"S3",UI_Graph_ADD,9,UI_Color_Orange,4,20,1670,838,8,"CONV");//传送带是否开启文字						静态
										Char_ReFresh(S3);
								}
								if (show_num == 6)//第7显示段***********************************************
								{			
										loop_end_flag = 0;//循环结束标志
										
										//Z进度条描边
										Line_Draw(&G19, "G19", UI_Graph_ADD, 8, UI_Color_White, 4, 1638, 752, 1638, 348);
										Line_Draw(&G20, "G20", UI_Graph_ADD, 8, UI_Color_White, 4, 1662, 752, 1662, 348);
										Line_Draw(&G21, "G21", UI_Graph_ADD, 8, UI_Color_White, 4, 1638, 752, 1662, 752);
										Line_Draw(&G22, "G22", UI_Graph_ADD, 8, UI_Color_White, 4, 1638, 348, 1662, 348);
										//将图形内容推送至缓存数组内
										UI_ReFresh(5,G19,G20,G21,G22);
								}
								if (show_num == 7)//第8显示段***********************************************
								{			
										loop_end_flag = 0;//循环结束标志
						

								}
								if (show_num == 8)//第9显示段***********************************************
								{			
										loop_end_flag = 0;//循环结束标志
						
										Char_Draw(&S6,"S6",UI_Graph_ADD,9,UI_Color_Green,8,40,685,900,15,"What can i say?");//666								静态
										Char_ReFresh(S6);
								}
								if (show_num == 9)//第10显示段***********************************************
								{			
										loop_end_flag = 0;//循环结束标志
						
										Char_Draw(&S7,"S7",UI_Graph_ADD,9,UI_Color_Orange,4,20,1670,788,8,"Keyboard");//按键切换文字								静态					
										Char_ReFresh(S7);
								}
					
								//显示段递增
								show_num++;			
								//UI推送段结束
								if (loop_end_flag == 1)
								{
										UI_init_flag = 0;
										show_num = 0;
								}
								loop_end_flag = 1;
						}
//UI动态显示内容**************************************************************************************************************
						else
						{
								//UI更新显示内容			
								if (show_num == 0)//第1显示段**********************************************
								{			
										loop_end_flag = 0;//循环结束标志
										
										
										
										Line_Draw(&G7,"G7",UI_Graph_Change,9,UI_Color_Purplish_red,20,710,820,710+map(x_distance,0,8191*114,0,500),820);					//X轴显示进度条	动态
									
										Line_Draw(&G8,"G8",UI_Graph_Change,9,UI_Color_Purplish_red,20,1650,350,1650,350+map(z_distance,0,8191*73,0,400));					//Z抬升显示进度条	动态
										

										if(keyboard_ctrl.keyboard_V_SHORT_flag == 1)									//传送带开启标志					绿色
											Circle_Draw(&G6,"G6",UI_Graph_Change,9,UI_Color_Green,10,1640,825,9);			
										else
											Circle_Draw(&G6,"G6",UI_Graph_Change,9,UI_Color_White,10,1640,825,9);
										
										if(keyboard_ctrl.turn_flag == 1)									//气泵开启标志				绿色
											Circle_Draw(&G9,"G9",UI_Graph_Del,9,UI_Color_Orange,5,957,538,80);				
										else
											Circle_Draw(&G9,"G9",UI_Graph_ADD,9,UI_Color_Orange,5,957,538,80);	
						
										if(keyboard_ctrl.keyboard_control_flag == 1)					//按键开启标志					绿色
											Circle_Draw(&G10,"G10",UI_Graph_Change,9,UI_Color_Green,10,1640,775,9);				
										else
											Circle_Draw(&G10,"G10",UI_Graph_Change,9,UI_Color_White,10,1640,775,9);
										
										//将图形内容推送至缓存数组内
										UI_ReFresh(7,G4,G5,G6,G7,G8,G9,G10);
								}
								if (show_num == 1)//第2显示段***********************************************
								{			
										loop_end_flag = 0;//循环结束标志

								}
					
								//显示段递增
								show_num++;			
								if (!keyboard_ctrl.keyboard_control_flag && show_num == 2)
								{
										//UI界面关闭
										keyboard_ctrl.UI_open_flag = 0;
								}
								//UI推送段全部结束了
								if (loop_end_flag == 1)
								{
										UI_init_flag = 0;
										show_num = 0;
								}
								loop_end_flag = 1;
								
								
						}
						
						usart6_tx_dma_enable(usart6_tx_buf,usart6_tx_len);				//将UI数据发送出去
						usart6_tx_len = 0;																				//长度清零
						
				}
				
		}
		else
		{		
				UI_init_flag = 1;
				UI_time = 0;
				
				show_num = 0;
		}
}



/********************************************删除操作*************************************
**参数：Del_Operate  对应头文件删除操作
        Del_Layer    要删除的层 取值0-9
*****************************************************************************************/

void UI_Delete(u8 Del_Operate,u8 Del_Layer)
{

   unsigned char *framepoint;                   //读写指针
   u16 frametail=0xFFFF;                        //CRC16校验值
   int loop_control;                       			//For函数循环控制
	
   UI_Packhead framehead;												
   UI_Data_Operate datahead;
   UI_Data_Delete del;
   
   framepoint=(unsigned char *)&framehead;
   
   framehead.SOF=UI_SOF;
   framehead.Data_Length=8;
   framehead.Seq=UI_Seq;
   framehead.CRC8=Get_CRC8_Check_Sum_UI(framepoint,4,0xFF);
   framehead.CMD_ID=UI_CMD_Robo_Exchange;                   //填充包头数据
   
   datahead.Data_ID=UI_Data_ID_Del;
   datahead.Sender_ID=Robot_ID;
   datahead.Receiver_ID=Cilent_ID;                          //填充操作数据
   
   del.Delete_Operate=Del_Operate;
   del.Layer=Del_Layer;                                     //控制信息
   
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(framehead),frametail);
   framepoint=(unsigned char *)&datahead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(datahead),frametail);
   framepoint=(unsigned char *)&del;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(del),frametail);  //CRC16校验值计算
   
	 //把所有数据放到发送缓存数组里
   framepoint=(unsigned char *)&framehead;
   for(loop_control=0;loop_control<sizeof(framehead);loop_control++)
   {
			usart6_tx_buf[usart6_tx_len]=*framepoint;					//数据放到发送缓存数组里
			usart6_tx_len++;			//发送数据长度+1
      framepoint++;
   }
   framepoint=(unsigned char *)&datahead;
   for(loop_control=0;loop_control<sizeof(datahead);loop_control++)
   {
      usart6_tx_buf[usart6_tx_len]=*framepoint;					//数据放到发送缓存数组里
			usart6_tx_len++;			//发送数据长度+1
      framepoint++;
   }
   framepoint=(unsigned char *)&del;
   for(loop_control=0;loop_control<sizeof(del);loop_control++)
   {
      usart6_tx_buf[usart6_tx_len]=*framepoint;					//数据放到发送缓存数组里
			usart6_tx_len++;			//发送数据长度+1
      framepoint++;
   }                                                                 //发送所有帧
   framepoint=(unsigned char *)&frametail;
   for(loop_control=0;loop_control<sizeof(frametail);loop_control++)
   {
      usart6_tx_buf[usart6_tx_len]=*framepoint;					//数据放到发送缓存数组里
			usart6_tx_len++;			//发送数据长度+1
      framepoint++;                                                  //发送CRC16校验值
   }
   
   UI_Seq++;                                                         //包序号+1
}
/************************************************绘制直线*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Start_x、Start_x    开始坐标
        End_x、End_y   结束坐标
**********************************************************************************************************/
        
void Line_Draw(UI_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 End_x,u32 End_y)
{
   int i;
   for(i=0;i<3&&imagename[i]!='\0';i++)
      image->graphic_name[2-i]=imagename[i];
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->end_x = End_x;
   image->end_y = End_y;
}

/************************************************绘制矩形*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Start_x、Start_x    开始坐标
        End_x、End_y   结束坐标（对顶角坐标）
**********************************************************************************************************/
        
void Rectangle_Draw(UI_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 End_x,u32 End_y)
{
   int i;
   for(i=0;i<3&&imagename[i]!='\0';i++)
      image->graphic_name[2-i]=imagename[i];
   image->graphic_tpye = UI_Graph_Rectangle;
	 image->graphic_tpye = UI_Graph_Line;
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->end_x = End_x;
   image->end_y = End_y;
}

/************************************************绘制整圆*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Start_x、Start_x    圆心坐标
        Graph_Radius  图形半径
**********************************************************************************************************/
        
void Circle_Draw(UI_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 Graph_Radius)
{
   int i;
   for(i=0;i<3&&imagename[i]!='\0';i++)
      image->graphic_name[2-i]=imagename[i];
   image->graphic_tpye = UI_Graph_Circle;
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->radius = Graph_Radius;
}

/************************************************绘制圆弧*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Graph_StartAngle,Graph_EndAngle    开始，终止角度
        Start_y,Start_y    圆心坐标
        x_Length,y_Length   x,y方向上轴长，参考椭圆
**********************************************************************************************************/
        
void Arc_Draw(UI_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 Graph_StartAngle,u32 Graph_EndAngle,u32 x_Length,u32 y_Length)
{
   int i;
   
   for(i=0;i<3&&imagename[i]!='\0';i++)
      image->graphic_name[2-i]=imagename[i];
   image->graphic_tpye = UI_Graph_Arc;
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->start_angle = Graph_StartAngle;
   image->end_angle = Graph_EndAngle;
   image->end_x = x_Length;
   image->end_y = y_Length;
}



/************************************************绘制整型数据**********************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Graph_Size     字号
        Start_x、Start_x    开始坐标
        Graph_Int   要显示的变量
**********************************************************************************************************/
        
void Int_Draw(UI_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Width,u32 Graph_Size,u32 Start_x,u32 Start_y,uint32_t Graph_Int)
{
   int i;

   for(i=0;i<3&&imagename[i]!='\0';i++)
      image->graphic_name[2-i]=imagename[i];
   image->graphic_tpye = UI_Graph_Int;
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->font_size = Graph_Size;
   image->graph_Int = Graph_Int;			
}



/************************************************绘制浮点型数据*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Graph_Size     字号
        Start_x、Start_x    开始坐标
        Graph_Float   要显示的变量
**********************************************************************************************************/
        
void Float_Draw(UI_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Width,u32 Graph_Size,u32 Start_x,u32 Start_y,float Graph_Float)
{
   int i;

   for(i=0;i<3&&imagename[i]!='\0';i++)
      image->graphic_name[2-i]=imagename[i];
   image->graphic_tpye = UI_Graph_Float;
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->font_size = Graph_Size;
   image->graph_Float = (uint32_t)(Graph_Float*1000);			//传入值需要*1000
}



/************************************************绘制字符型数据*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   			图片名称，用于标识更改
        Graph_Operate   		图片操作，见头文件
        Graph_Layer    			图层0-9
        Graph_Color    			图形颜色
        Graph_Width    			图形线宽
        Graph_Size     			字号
        Graph_Digit    			字符个数
        Start_x、Start_x    开始坐标
        *Char_Data          待发送字符串开始地址
**********************************************************************************************************/
        
void Char_Draw(String_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Width,u32 Graph_Size,u32 Start_x,u32 Start_y,u32 Graph_Digit,char *Char_Data)
{
   int i;
   
   for(i=0;i<3&&imagename[i]!='\0';i++)
      image->Graph_Control.graphic_name[2-i]=imagename[i];
   image->Graph_Control.graphic_tpye = UI_Graph_Char;
   image->Graph_Control.operate_tpye = Graph_Operate;
   image->Graph_Control.layer = Graph_Layer;
   image->Graph_Control.color = Graph_Color;
   image->Graph_Control.width = Graph_Width;
   image->Graph_Control.start_x = Start_x;
   image->Graph_Control.start_y = Start_y;
   image->Graph_Control.font_size = Graph_Size;
   image->Graph_Control.str_len = Graph_Digit;
   
   for(i=0;i<Graph_Digit;i++)
   {
      image->show_Data[i]=*Char_Data;
      Char_Data++;
   }
}

/************************************************UI推送函数（使更改生效）*********************************
**参数： cnt   图形个数
         ...   图形变量参数


Tips：：该函数只能推送1，2，5，7个图形，其他数目协议未涉及
**********************************************************************************************************/
int UI_ReFresh(int cnt,...)
{
   int i,n;
   UI_Data imageData;
   unsigned char *framepoint;                      //读写指针
   u16 frametail=0xFFFF;                        //CRC16校验值
	
	
   UI_Packhead framehead;         //帧头
   UI_Data_Operate datahead;      //数据头
   
   va_list ap;
   va_start(ap,cnt);
   
   framepoint=(unsigned char *)&framehead;
   framehead.SOF=UI_SOF;
   framehead.Data_Length=6+cnt*15;
   framehead.Seq=UI_Seq;
   framehead.CRC8=Get_CRC8_Check_Sum_UI(framepoint,4,0xFF);
   framehead.CMD_ID=UI_CMD_Robo_Exchange;                   //填充包头数据
   
   switch(cnt)
   {
      case 1:
         datahead.Data_ID=UI_Data_ID_Draw1;
         break;
      case 2:
         datahead.Data_ID=UI_Data_ID_Draw2;
         break;
      case 5:
         datahead.Data_ID=UI_Data_ID_Draw5;
         break;
      case 7:
         datahead.Data_ID=UI_Data_ID_Draw7;
         break;
      default:
         return (-1);
   }
   datahead.Sender_ID=Robot_ID;
   datahead.Receiver_ID=Cilent_ID;                          //填充操作数据
   
   framepoint=(unsigned char *)&framehead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(framehead),frametail);
   framepoint=(unsigned char *)&datahead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(datahead),frametail);          //CRC16校验值计算（部分）
   
   framepoint=(unsigned char *)&framehead;      //帧头
   for(i=0;i<sizeof(framehead);i++)
   {
      usart6_tx_buf[usart6_tx_len]=*framepoint;					//数据放到发送缓存数组里
			usart6_tx_len++;			//发送数据长度+1
      framepoint++;
   }
   framepoint=(unsigned char *)&datahead;       //数据头
   for(i=0;i<sizeof(datahead);i++)
   {
      usart6_tx_buf[usart6_tx_len]=*framepoint;					//数据放到发送缓存数组里
			usart6_tx_len++;			//发送数据长度+1
      framepoint++;
   }
   
   for(i=0;i<cnt;i++)            
   {
      imageData=va_arg(ap,UI_Data);
      
      framepoint=(unsigned char *)&imageData;
      frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(UI_Data),frametail);             //CRC16校验
      
      for(n=0;n<sizeof(UI_Data);n++)
      {
         usart6_tx_buf[usart6_tx_len]=*framepoint;					//数据放到发送缓存数组里
			   usart6_tx_len++;			//发送数据长度+1
         framepoint++;             
      }                                               //图形帧
   }
   framepoint=(unsigned char *)&frametail;
   for(i=0;i<sizeof(frametail);i++)
   {
      usart6_tx_buf[usart6_tx_len]=*framepoint;					//数据放到发送缓存数组里
			usart6_tx_len++;			//发送数据长度+1
      framepoint++;                                                  //CRC16校验值
   }
   
   va_end(ap);
   
   UI_Seq++;                                                         //包序号+1
   return 0;
}


/************************************************UI推送字符（使更改生效）*********************************/
int Char_ReFresh(String_Data string_Data)
{
   int i;
   String_Data imageData;
   unsigned char *framepoint;                      //读写指针
   u16 frametail=0xFFFF;                        //CRC16校验值
	
	
	
   UI_Packhead framehead;
   UI_Data_Operate datahead;
   imageData=string_Data;
   
   
   framepoint=(unsigned char *)&framehead;
   framehead.SOF=UI_SOF;
   framehead.Data_Length=6+45;
   framehead.Seq=UI_Seq;
   framehead.CRC8=Get_CRC8_Check_Sum_UI(framepoint,4,0xFF);
   framehead.CMD_ID=UI_CMD_Robo_Exchange;                   //填充包头数据
   

   datahead.Data_ID=UI_Data_ID_DrawChar;

   datahead.Sender_ID=Robot_ID;
   datahead.Receiver_ID=Cilent_ID;                          //填充操作数据
   
   framepoint=(unsigned char *)&framehead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(framehead),frametail);
   framepoint=(unsigned char *)&datahead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(datahead),frametail);
   framepoint=(unsigned char *)&imageData;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(imageData),frametail);             //CRC16校验   //CRC16校验值计算（部分）
   
   framepoint=(unsigned char *)&framehead;
   for(i=0;i<sizeof(framehead);i++)
   {
      usart6_tx_buf[usart6_tx_len]=*framepoint;					//数据放到发送缓存数组里
			usart6_tx_len++;			//发送数据长度+1
      framepoint++;
   }
   framepoint=(unsigned char *)&datahead;
   for(i=0;i<sizeof(datahead);i++)
   {
      usart6_tx_buf[usart6_tx_len]=*framepoint;					//数据放到发送缓存数组里
			usart6_tx_len++;			//发送数据长度+1
      framepoint++;
   }                                                   //操作数据  
   framepoint=(unsigned char *)&imageData;
   for(i=0;i<sizeof(imageData);i++)
   {
      usart6_tx_buf[usart6_tx_len]=*framepoint;					//数据放到发送缓存数组里
			usart6_tx_len++;			//发送数据长度+1
      framepoint++;             
   }                                               //图形帧
   
   
   
   framepoint=(unsigned char *)&frametail;
   for(i=0;i<sizeof(frametail);i++)
   {
      usart6_tx_buf[usart6_tx_len]=*framepoint;					//数据放到发送缓存数组里
			usart6_tx_len++;			//发送数据长度+1
      framepoint++;                                                  //CRC16校验值
   }
   
   UI_Seq++;                                                         //包序号+1
   return 0;
}


/*****************************************************CRC8校验值计算**********************************************/
const unsigned char CRC8_INIT_UI = 0xff; 
const unsigned char CRC8_TAB_UI[256] = 
{ 
0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41, 
0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc, 
0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62, 
0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff, 
0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07, 
0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a, 
0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24, 
0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9, 
0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd, 
0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50, 
0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee, 
0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73, 
0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b, 
0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16, 
0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8, 
0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35, 
};
unsigned char Get_CRC8_Check_Sum_UI(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8) 
{ 
	unsigned char ucIndex; 
	while (dwLength--) 
	{ 
		ucIndex = ucCRC8^(*pchMessage++); 
		ucCRC8 = CRC8_TAB_UI[ucIndex]; 
	} 
	return(ucCRC8); 
}

uint16_t CRC_INIT_UI = 0xffff; 
const uint16_t wCRC_Table_UI[256] = 
{ 
0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 
0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 
0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e, 
0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 
0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd, 
0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5, 
0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 
0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974, 
0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb, 
0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 
0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72, 
0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 
0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 
0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738, 
0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 
0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff, 
0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 
0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 
0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5, 
0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 
0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134, 
0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c, 
0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 
0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb, 
0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232, 
0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 
0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1, 
0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9, 
0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 
0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};
/* 
** Descriptions: CRC16 checksum function 
** Input: Data to check,Stream length, initialized checksum 
** Output: CRC checksum 
*/ 
uint16_t Get_CRC16_Check_Sum_UI(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC) 
{ 
	Uint8_t chData; 
	if (pchMessage == NULL) 
	{ 
		return 0xFFFF; 
	} 
	while(dwLength--) 
	{ 
		chData = *pchMessage++;
		(wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table_UI[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff]; 
	} 
	return wCRC; 
}




