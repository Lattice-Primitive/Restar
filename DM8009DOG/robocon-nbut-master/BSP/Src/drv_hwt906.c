#include "drv_hwt906.h"

void HWT906_I2C_SPI_Reboot(HWT906_I2C_SPI_t* hwt906)
{
    uint8_t reboot_cmd[]={0x88,0xb5};//解锁
    hwt906->opt->HWT906_WriteData(hwt906->addr,0x69,reboot_cmd,2);
//		reboot_cmd[0] = 0xff;//恢复出厂设置
//		reboot_cmd[1] = 0x00;
//    hwt906->opt->HWT906_WriteData(hwt906->addr,0x00,reboot_cmd,2);
//		reboot_cmd[0] = 0x88;
//		reboot_cmd[1] = 0xb5;
//		hwt906->opt->HWT906_WriteData(hwt906->addr,0x69,reboot_cmd,2);
	  reboot_cmd[0] = 0x01;//设置6轴算法
		reboot_cmd[1] = 0x00;
		hwt906->opt->HWT906_WriteData(hwt906->addr,0x24,reboot_cmd,2);
	  HWT906_I2C_SPI_Correction(hwt906,1000);
}

void HWT906_I2C_SPI_Correction(HWT906_I2C_SPI_t* hwt906,uint32_t ticks)
{
    uint8_t corr_cmd[]={0x88,0xb5};//解锁
    hwt906->opt->HWT906_WriteData(hwt906->addr,0x69,corr_cmd,2);
    corr_cmd[0]=0x01;//加速度计校准
    corr_cmd[1]=0x00;
    hwt906->opt->HWT906_WriteData(hwt906->addr,0x01,corr_cmd,2);
    corr_cmd[0]=0x04;//航向角置零
		hwt906->opt->HWT906_WriteData(hwt906->addr,0x01,corr_cmd,2);
		corr_cmd[0]=0x00;//正常工作模式
		hwt906->opt->HWT906_Delay_Ms(ticks);
		if(HWT906_I2C_SPI_ReadOrderData(hwt906, HWT906_ANGLE_Z) != 0x00)
		{
				HWT906_I2C_SPI_Reboot(hwt906);
		}
    hwt906->opt->HWT906_WriteData(hwt906->addr,0x01,corr_cmd,2);
}

void HWT906_I2C_SPI_ReadAllData(HWT906_I2C_SPI_t* hwt906,HWT906_DataPack_t* pack)
{
    uint8_t read_data[8];
		//加速度
    hwt906->opt->HWT906_ReadData(hwt906->addr,0x34,read_data,6);
    pack->accel->x=((int16_t)read_data[1]<<8)|read_data[0];
    pack->accel->y=((int16_t)read_data[3]<<8)|read_data[2];
    pack->accel->z=((int16_t)read_data[5]<<8)|read_data[4];
    //角速度
    hwt906->opt->HWT906_ReadData(hwt906->addr,0x37,read_data,6);
    pack->gyro->x=((int16_t)read_data[1]<<8)|read_data[0];
    pack->gyro->y=((int16_t)read_data[3]<<8)|read_data[2];
    pack->gyro->z=((int16_t)read_data[5]<<8)|read_data[4];
    //磁场
    hwt906->opt->HWT906_ReadData(hwt906->addr,0x3a,read_data,6);
    pack->magnet->x=((int16_t)read_data[1]<<8)|read_data[0];
    pack->magnet->y=((int16_t)read_data[3]<<8)|read_data[2];
    pack->magnet->z=((int16_t)read_data[5]<<8)|read_data[4];
    //横滚角 俯仰角 航向角
    hwt906->opt->HWT906_ReadData(hwt906->addr,0x3d,read_data,6);
    pack->angle->x=((int16_t)read_data[1]<<8)|read_data[0];
    pack->angle->y=((int16_t)read_data[3]<<8)|read_data[2];
    pack->angle->z=((int16_t)read_data[5]<<8)|read_data[4];
    //温度
    hwt906->opt->HWT906_ReadData(hwt906->addr,0x40,read_data,2);
    pack->temp=((int16_t)read_data[1]<<8)|read_data[0];
    //四元数
    hwt906->opt->HWT906_ReadData(hwt906->addr,0x51,read_data,8);
    pack->quat[0]=((int16_t)read_data[1]<<8)|read_data[0];
    pack->quat[1]=((int16_t)read_data[3]<<8)|read_data[2];
    pack->quat[2]=((int16_t)read_data[5]<<8)|read_data[4];
    pack->quat[3]=((int16_t)read_data[7]<<8)|read_data[6];
}

int16_t HWT906_I2C_SPI_ReadOrderData(HWT906_I2C_SPI_t* hwt906,HWT906_DataType_t data_type)
{
    uint8_t data[2];
    data[0]=(uint8_t)data_type;
    hwt906->opt->HWT906_ReadData(hwt906->addr,data[0],data,2);
    return ((int16_t)data[1]<<8)|data[0];
}
