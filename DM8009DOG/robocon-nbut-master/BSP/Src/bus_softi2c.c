#include "bus_softi2c.h"

void SoftI2C_Start(SoftI2C_Operate_t* i2c_opt)
{
    // 时钟线高电平时数据线下降沿
    i2c_opt->SDA_OUT();
    i2c_opt->SDA_H();
    i2c_opt->SCL_H();
    i2c_opt->Delay_Us(4);
    i2c_opt->SDA_L();
    i2c_opt->Delay_Us(4);
    i2c_opt->SCL_L();
}

void SoftI2C_Stop(SoftI2C_Operate_t* i2c_opt)
{
    // 时钟线高电平时数据线上升沿
    i2c_opt->SDA_OUT();
    i2c_opt->SCL_L();
    i2c_opt->SDA_L();
    i2c_opt->Delay_Us(4);
    i2c_opt->SCL_H();
    i2c_opt->Delay_Us(2);
    i2c_opt->SDA_H();
    i2c_opt->Delay_Us(4);
}

SoftI2C_IsOK_t SoftI2C_WaitACK(SoftI2C_Operate_t* i2c_opt)
{
    // 时钟线保持低电平，数据线拉低时为ACK
    uint8_t Err_Time=0; //等待时间
    i2c_opt->SDA_IN();
    i2c_opt->SDA_H();
    i2c_opt->Delay_Us(1);
    i2c_opt->SCL_H();
    i2c_opt->Delay_Us(1);
    while(i2c_opt->SDA_R()) //轮询检测电平
    {
        Err_Time++;
        if(Err_Time>250) //超过一定时间
        {
            SoftI2C_Stop(i2c_opt); //发送停止信号
            return SoftI2C_NOK; //发送失败
        }
    }
    i2c_opt->SCL_L(); //钳住总线
    return SoftI2C_OK;
}

void SoftI2C_ACK(SoftI2C_Operate_t* i2c_opt)
{
    // 时钟线保持低电平，数据线拉低时为ACK
    i2c_opt->SCL_L();
    i2c_opt->Delay_Us(1);
    i2c_opt->SDA_OUT();
    i2c_opt->SDA_L();
    i2c_opt->Delay_Us(2);
    i2c_opt->SCL_H();
    i2c_opt->Delay_Us(2);
    i2c_opt->SCL_L();
}

void SoftI2C_NACK(SoftI2C_Operate_t* i2c_opt)
{
    // 时钟线保持低电平，数据线拉高时为NACK
    i2c_opt->SCL_L();
    i2c_opt->SDA_OUT();
    i2c_opt->SDA_H();
    i2c_opt->Delay_Us(2);
    i2c_opt->SCL_H();
    i2c_opt->Delay_Us(2);
    i2c_opt->SCL_L();
}

void SoftI2C_Write_Byte(SoftI2C_Operate_t* i2c_opt, uint8_t txd)
{
    uint8_t t;
    i2c_opt->SDA_OUT();
    i2c_opt->SCL_L(); 
    for(t=0;t<8;t++)
    {
        if((txd&0x80)>>7) //高位先出
        {
            i2c_opt->SDA_H();
        }
        else
        {
            i2c_opt->SDA_L();
        }
        txd<<=1;
        i2c_opt->Delay_Us(2);
        i2c_opt->SCL_H(); //高电平采样
        i2c_opt->Delay_Us(2);
        i2c_opt->SCL_L();
        i2c_opt->Delay_Us(2);
    }
}

uint8_t SoftI2C_Read_Byte(SoftI2C_Operate_t* i2c_opt)
{
    uint8_t i, rxd = 0;
    i2c_opt->SDA_IN();
    for(i = 0; i < 8; i++)
    {
        i2c_opt->SCL_L();
        i2c_opt->Delay_Us(2);
        i2c_opt->SCL_H(); //高电平采样
         rxd <<= 1;
        if (i2c_opt->SDA_R())
            rxd++;
        i2c_opt->Delay_Us(1);
    }
    return rxd;
}

SoftI2C_IsOK_t SoftI2C_Write_Mem(SoftI2C_Operate_t* i2c_opt, SoftI2C_RW_Mem_t* rw_mem)
{
    uint32_t i;
    if((rw_mem->data_size<1)||(rw_mem->pdata==0))// 发送的数据不合法
    {
        return SoftI2C_NOK;
    }
    SoftI2C_Start(i2c_opt);
    SoftI2C_Write_Byte(i2c_opt,(rw_mem->slave_addr<<1)); //发送从机地址与发送指令
    if(SoftI2C_NOK==SoftI2C_WaitACK(i2c_opt)) //等待响应
    {
        return SoftI2C_NOK;
    }
    SoftI2C_Write_Byte(i2c_opt,rw_mem->slave_reg); //发送寄存器地址
    for(i=0;i<rw_mem->data_size;i++) //发送数据
    {
        if(SoftI2C_NOK==SoftI2C_WaitACK(i2c_opt))
        {
            return SoftI2C_NOK;
        }
        SoftI2C_Write_Byte(i2c_opt,rw_mem->pdata[i]);
    }
    if(SoftI2C_NOK==SoftI2C_WaitACK(i2c_opt))
    {
        return SoftI2C_NOK;
    }
    SoftI2C_Stop(i2c_opt);
    return SoftI2C_OK;
}

SoftI2C_IsOK_t SoftI2C_Read_Mem(SoftI2C_Operate_t* i2c_opt, SoftI2C_RW_Mem_t* rw_mem)
{
    uint32_t i;
    if((rw_mem->data_size<1)||(rw_mem->pdata==0)) // 发送的数据不合法
    {
        return SoftI2C_NOK;
    }
    SoftI2C_Start(i2c_opt);
    SoftI2C_Write_Byte(i2c_opt,(rw_mem->slave_addr<<1)); // 发送从机地址与发送指令
    if(SoftI2C_NOK==SoftI2C_WaitACK(i2c_opt))
    {
        return SoftI2C_NOK;
    }
    SoftI2C_Write_Byte(i2c_opt,rw_mem->slave_reg); // 发送寄存器地址
    if(SoftI2C_NOK==SoftI2C_WaitACK(i2c_opt))
    {
        return SoftI2C_NOK;
    }
    SoftI2C_Start(i2c_opt);
    SoftI2C_Write_Byte(i2c_opt,(rw_mem->slave_addr<<1|0x01)); // 发送从机地址与接收指令
    if(SoftI2C_NOK==SoftI2C_WaitACK(i2c_opt))
    {
        return SoftI2C_NOK;
    }
    for(i=0;i<rw_mem->data_size-1;i++) //接收数据
    {
        rw_mem->pdata[i]=SoftI2C_Read_Byte(i2c_opt);
        SoftI2C_ACK(i2c_opt);
    }
    rw_mem->pdata[i]=SoftI2C_Read_Byte(i2c_opt);
    SoftI2C_NACK(i2c_opt);

    SoftI2C_Stop(i2c_opt);
    return SoftI2C_OK;
}
