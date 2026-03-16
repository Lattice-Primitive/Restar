# 电控人程序规范草案(Elite Embeded Programing Standard alpha)

作者：Zhiyuan_Mao 电科192毛致远

gitee：<https://gitee.com/zhiyuan-mao/eeps_2021>

## 目录
___

* [前言](#1)
* [1.IDE与工具链](#2)
* [2.编程细节规范](#3)
* [3.编程风格规范](#4)
* [4.其他规范](#5)
___
<h2 id="1">前言</h2>

这几天一直在琢磨，如何写出更加规整的程序。我看了看近期我所写的代码，从**微观角度**来看，似乎除了注释稍微少了一些，也没太大毛病，函数接口都说的比较明白，该注意的一些问题都注意了。但是还是给我一种“有点乱”、“冗余”的感觉，比如
```c
void Serial_Println(Serial_t *serial,char *data)
{
    while (*data)
    {
        Serial_TransmitByte(serial,*data);
        data += 1;
    }
    Serial_TransmitByte(serial,'\r');
    Serial_TransmitByte(serial,'\n');
}

void Serial_Print(Serial_t *serial,char *data)
{
    while (*data)
    {
        Serial_TransmitByte(serial,*data);
        data++;
    }
}

```
明明多加一个输入参数就可以解决的问题，非要整这么多花里胡哨的，搁这冲KPI呢。(~~别尬黑，这波操作减少了对栈空间的使用~~)

还有就是分层问题，驱动层和应用层的代码放在一起，把用户整不会了。
```c
 /**
 * @brief 向oled屏发送多个字节数据
 * @param oled (OLED_t*)OLED结构体指针
 * @param oled_pdata (uint8_t*) 发送给oled的数据指针
 * @param data_size (uint32_t) 发送给oled的数据大小
 * @return 是否发送成功(uint8_t) 0为成功，1为失败
 * */
    extern uint8_t OLED_Send_DataMultiBytes(OLED_t *oled, uint8_t *oled_pdata, uint32_t data_size);

    /**
 * @brief 开启oled屏显示
 * @param oled (OLED_t*)OLED结构体指针
 * @return NULL
 * */
    extern void OLED_Display_On(OLED_t *oled);
```
所以我们到底该用哪个接口？还是一起用？

有些写法给用户带来了很大的麻烦，比如
```c
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "delay.h"
#include "serial.h"
#include "sys_setting.h"

uint8_t ch=0;
Serial_t Serial2;

int main(void)
{		
 	WDT_A_holdTimer();
	System_Init();
	delay_init();
	Serial_Init_Default(&Serial2);//初始化设备结构体
  	Serial_Init(EUSCI_A2_BASE,&Serial2);//初始化设备
	Serial_Stdio_Set(&Serial2);//初始化stdio库使用
	while(1)
	{
		...//省略
	}
}

```
3个初始化，漏一个就出问题，为啥不能一步到位？

这种离谱的写法还能在我代码里找到很多。这样的代码对维护者和开发者来说都是折磨。

再加上电控组整体水平都参差不齐，有些学的快的已经差不多掌握了如何开发ARM架构的芯片，学的慢的可能对c语言的认识都不是特别深。

所以需要一种标准来规范一个程序应该怎么写，规范的编码可以增加彼此的工作效率，更加方便传承和迭代。~~苦了一开始的开发人员，全得重新按照规范来，写起来会比之前累一点~~。如果能够推广的话还是利大于弊的。

这个标准会从IDE与工具链、编程细节规范、编程风格规范三个大方面来说明。基本涵盖了电控人的写代码过程。该标准还是以MCU开发为主。

我会用三种标签表示需要遵守的等级：

<font color="red">强制</font> ：表示必须要遵守的规定

<font color="yellow">推荐</font> ：表示最好遵守的规定

___

<h2 id="2">1.IDE与工具链</h2>

1.1 <font color="red">强制</font> 在一般的*微控制器*(MCU,如MCS-51，ARM Cortex-M)开发的情况下尽量使用*C语言*和*汇编语言*，除以下情形：Arduino开发，OpenMV等主流使用C++与Python语言的除外

PS:最近有不止一位同学提出要使用C++为MCU编程。目前的MCU主流语言还是C语言，主要是因为C语言更加接近底层，效率高，易学，便于调试。C++因为封装和继承特性受一些软件开发者的青睐。

经过研判，目前我们所能看到的电控代码几乎全是基于C语言的，我在github上做过估计，大约40个仓库中，只有一个仓库是使用C++的，其余均是C语言，并且我们有幸得到的robocon开源电控代码也是基于C的，可见C语言仍是且将长期是MCU主流语言，而且C语言各位几乎都会。我也会继续推荐C语言的使用，这样方便与其他学校进行学习交流。

但也不会完全禁止C++或Python，但主流肯定还是C。

    But mind you, writing complex code in C can give you nightmares.

    But then debugging C++ code can give you nightmares as well.

（摘自博客<https://blog.csdn.net/NoDistanceY/article/details/104580751>的引用）

不过得益于ARMCC编译器的强大，C与C++编译出来的程序在大小和效率方面都差不了多少，实测C++也就比C大了10%都不到的空间，调试其实问题也并不大。

1.2 <font color="red">强制</font> 在*ARM Cortex-M*开发中我们最好使用*KeilMDK*作为我们的IDE，在*Arduino*开发中使用Arduino官网提供的IDE。

PS:在前几个月前有一个要彻底废除KeilMDK的想法，我之前也在Ubuntu18.04的环境下也成功了。但后面调研发现国内外主流还是以KeilMDK为主，github上的MCU项目也是KeilMDK居多，在上次去Robocon之行中发现几乎所有的学校都使用了KeilMDK。而且根据实际体验，使用KeilMDK也的确比使用GCC配合OpenOCD要方便和好用不少，唯一缺点就是界面不行和闭源。因此近几年还是以KeilMDK为主。

1.3 <font color="yellow">推荐</font> 使用VSCode来完成代码的编写与查看。

PS:VSCode推荐插件：

    1. Better C++ Syntax
    2. Better Comments
    3. Bracket Pair Colorizer
    4. Chinese (Simplified) Language Pack for Visual Studio Code
    5. TODO Highlight v2
    6. vscode-icons
    ...

___
<h2 id="3">2.编程细节规范</h2>

2.1 <font color="red">强制</font> 代码的规范性受MISRA-C 2004约束。

[MISRA-C 2004 中文版](./MISRA-C-2004_工业标准的C编程规范_中文版.pdf)

[MISRA-C 2004 英文版](./MISRA_C_2004.pdf)

[MISRA-C 2012 部分中英对照版](./MISRA_C_2012.pdf)

[MISRA-C++ 2008 英文版](./MISRA_CPP_2008.pdf)

PS:最新的MISRA-C 2012有200+页，而且具体规则没有中文，就还是先使用2004年的标准吧，其实也够用了。

推荐看一下这篇blog<https://freertos.blog.csdn.net/article/details/45508029>，这篇讲了很多关于嵌入式C相关的规范和技巧。

我很喜欢这位博主<https://freertos.blog.csdn.net/article/list/1>他在FreeRTOS和嵌入式方面讲的很好

或者是洛克希德马丁(Lockheed Martin)公司出品，用于战斗机系统的编程标准[JSF-AV-rules](./JSF-AV-rules.pdf)
___
<h2 id="4">3.编程风格规范</h2>
3.1 <font color="red">强制</font>  代码整体的编写风格为华为编程规范，但会有一些差异在下面提及

[华为编程规范](./华为编程规范和范例.doc)

PS:菊花厂不多解释了，YYDS

3.2 <font color="yellow">推荐</font> 在华为的规定2-2,2-3(P8)条中(下面的编号形如x-x若未特别提及则为华为的规定)，对头注释过于严格，只需标出文件名、作者、描述即可。
如：
```c
/**
 * @file serial.h \c 串口通信模块 功能函数头文件
 * @author Zhiyuan Mao
 * @note
 * 该串口通信库实现了串口通信的基本收发功能，目前在 \b HC-05 模块上试验成功，HC-06
 * 暂时还没试过,并且只考虑了launchpad开发板上的UCA2RXD(P3.2),UCA2TXD(P3.3)
 **/
```
可以使用@，\ 等操作，使我们的注释更加漂亮，清晰。当然，要装VSCode插件(Better Comments)。

3.3 <font color="yellow">推荐</font> 2-4(P9)中对函数头部的注释过于严格，只需标出函数的基本使用方法、输入参数、返回参数即可，但需要说明输入和返回参数的类型，如果填入参数是指定的枚举型、宏定义，最好将可选值列出来，也可以像是这种形式：EUSCI_Ax_BASE (x=0，1，2)，最低标准是能让用户不看源文件就知道填什么。
如：
```c
    /**
 * @brief 串口通信初始化
 * @param module (uint32_t)模块名,可选值EUSCI_Ax_BASE (x=0，1，2)
 * @param serial (Serial_t *)串口通信结构体
 * @return (void)NULL
 * */
```

3.5 <font color="yellow">推荐</font> 3-5(P17) 函数命名方式还是使用STM32HAL库的形式：
```c
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_EnableListen_IT(I2C_HandleTypeDef *hi2c);
```
采用下划线+单词首字母大写的形式，看起来更加舒服。

函数名命名格式为：外设或设备名称_(操作主体或其属性)_操作名称(及被操作对象或其属性) _ (操作方式)

示例：(以串口通信和软件I2C为例)
```c
void Serial_Init(); //外设名称_操作名称() 串口通信初始化
void SoftI2C_Init();//外设名称_操作名称() 软件I2C初始化
void Serial_ReadByte();//外设名称_操作名称及被操作对象属性() 串口通信读取
void SoftI2C_Master_Read();//外设名称_操作主体属性_操作名称() 软件I2C作为主机读取
void Serial_Read_IT();//外设名称_操作名称被操作对象属性_操作方式() 串口通信以中断方式读取 
void SoftI2C_Master_Read_IT();//外设名称_操作主体属性_操作名称被操作对象属性_操作方式()软件I2C作为主机以中断方式读取
```
函数的参数最好也按照一定的顺序，通常有一定的逻辑与经验性

以一个OLED显示屏绘制字符串函数为例：
```c
   /**
 * @brief oled字符串绘制
 * @param oled (OLED_t*)OLED结构体指针
 * @param x (uint32_t)顶点(字符左上角)横坐标位置
 * @param y (uint32_t)顶点(字符左上角)纵坐标位置
 * @param pstr (uint8_t)字符串指针
 * @param str_size(uint32_t)字符串长度
 * @param font_size (uint8_t)字体大小
 * @param isfilled (uint8_t)是否填充
 * */
    extern void OLED_Gram_DrawString(OLED_t *oled, uint32_t x, uint32_t y, uint8_t*pstr, uint32_t str_size,uint8_t font, uint8_t isfilled);
```
一般第一个参数都是能够**代表外设主体**的结构体或其他参数

然后就是**要操作的对象**，这里就是一个xy坐标

之后是**操作的具体内容及属性**，这里指的是字符串的指针、字符串的大小、字体大小和是否填充。

最好如果有的话就是**操作的附加条件**，比如如果有阻塞限时(timeout)的话可以放在最后

3.6 <font color="yellow">推荐</font> 1/2 5-14(P26) 默认使用stdint.h库获得标准的整型。

3.7 <font color="red">强制</font>  宏定义的命名参照STM32HAL库的形式：
```c++
#define HAL_CAN_ERROR_NONE            (0x00000000U)  /*!< No error                                             */
#define HAL_CAN_ERROR_EWG             (0x00000001U)  /*!< Protocol Error Warning                               */
```
采用下划线+全大写的形式

宏定义命名格式为：外设或设备名称_目标属性名_属性状态

宏定义宜使用于状态掩膜中，即与位操作相关的常数，常用于外设寄存器的写入和判断。

3.8  <font color="red">强制</font>  枚举成员的命名参照STM32HAL库的形式：
```c++
typedef enum
{
  HAL_CAN_STATE_RESET             = 0x00U,  /*!< CAN not yet initialized or disabled*/
  HAL_CAN_STATE_READY             = 0x01U,  /*!< CAN initialized and ready for use*/
  HAL_CAN_STATE_LISTENING         = 0x02U,  /*!< CAN receive process is ongoing*/
  HAL_CAN_STATE_SLEEP_PENDING     = 0x03U,  /*!< CAN sleep request is pending*/
  HAL_CAN_STATE_SLEEP_ACTIVE      = 0x04U,  /*!< CAN sleep mode is active*/
  HAL_CAN_STATE_ERROR             = 0x05U   /*!< CAN error state*/

} HAL_CAN_StateTypeDef;
```
采用下划线+全大写的形式

枚举成员命名格式为：外设或设备名称_目标属性名_属性状态

枚举类型宜使用于状态分支判断中，多用于分支的状态判断中。

3.9  <font color="red">强制</font> 枚举、结构体都要采用typedef形式，如上条(3.8)的示例代码所示，名字由CAN_StateTypeDef改为CAN_State_t

3.10  <font color="yellow">推荐</font> 尽量使用宏函数和内联函数，提高效率,且宏函数和内联函数都遵守3.5

3.11 <font color="yellow">推荐</font> 关于驱动接口结构有一个提案，就是借鉴Linux的总线-设备-驱动结构，去掉设备层，再增加应用层，形成总线、驱动、应用三层结构，我以MPU6050驱动程序举例来说明三层的具体作用：

* 总线层(bus)：这里的总线其实泛指外设，包括GPIO、UART、I2C等*外设的初始化、基本的使用*，一般情况下已经被官方的库搞定了(如果没有官方库只有寄存器，那辛苦一下了，如果官方的外设不行或甚至没有这个外设，那么使用GPIO模拟的过程(比如软件I2C的)或与外部协议转换模块(比如SPI转CAN总线模块的SPI与模块的通信部分，但不涉及CAN总线与CAN总线设备之间的通信))。比如I2C外设的初始化和基础使用属于总线层

* 驱动层(driver)：驱动层一般是*外设的初始化与基本使用*，一般只是完成低层目的的一些步骤，比较简单，实现的是承上启下的作用。比如与MPU6050进行I2C协议的寄存器修改读取过程。

* 应用层(APP)：应用层一般是对驱动层接口的使用并实现对外设的使用。用户一般使用的就是应用层。比如MPU6050的初始化设置、数据读取及处理。**MPU6050的姿态解算算法和滤波算法部分不是应用层，但包括姿态解算和滤波过程的数据处理是应用层**，有些原理简单的驱动接口可以没有应用层(比如GPIO点灯,驱动层基本上就解决了所有问题)，有些驱动接口的应用层是非常灵活的，需要根据需求灵活改变，也可以无需应用层(比如红外反射传感器(循迹传感器)，在不同场合下的需求都是需要调整的)。

3.12 <font color="red">强制</font> 在头文件里的头尾添加以下内容：
```c++
#ifdef __cplusplus
extern "C" {
#endif

/*要写的程序*/

#ifdef __cplusplus
}
#endif
```
与C++可以兼容，将来若C++成为主流，这个库就还是可以用的。
___
<h2 id="5">4.其他规范</h2>

4.1 <font color="red">强制</font> 大型项目(比赛)写程序之前使用Xmind或其他软件绘制程序的结构图、说明使用的变量、函数及其参数。切忌盲目写程序

4.2 <font color="red">强制</font> 多人协作的程序要使用git

4.3 <font color="yellow">推荐</font> 项目期间做好每日的记录工作，使用Markdown。

4.4 <font color="red">强制</font> 库文件的命名格式：
层级(bus(总线)、drv(驱动)、app(应用))_外设或设备名.h(c)

如：

    bus_softi2c.h
    bus_softi2c.c
    drv_mpu6050.h
    drv_mpu6050.c
    app_mpu6050.h
    app_mpu6050.c

4.5 <font color="red">强制</font> 一个库(不管是硬件驱动库还是软件算法库)的文件夹下必须分为Inc(头文件)和Src(源文件)

4.6 <font color="yellow">推荐</font> 一个库(不管是硬件驱动库还是软件算法库)最好有用于介绍的.md文件

4.7 <font color="red">强制</font> 一个项目文件夹下要有一个.md文件，大致介绍整个项目

4.8 <font color="red">强制</font> 上述规则自发布起生效，已经开始的项目不受其约束。19级前的实验室成员不受其约束（除了作者以外），私人的代码不受其约束。
