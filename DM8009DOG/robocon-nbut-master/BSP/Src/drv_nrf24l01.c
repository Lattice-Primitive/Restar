#include "drv_nrf24l01.h"

#define NRF24L01_READ_REG        0x00
#define NRF24L01_WRITE_REG       0x20
#define NRF24L01_RD_RX_PLOAD     0x61
#define NRF24L01_WR_TX_PLOAD     0xA0
#define NRF24L01_FLUSH_TX        0xE1
#define NRF24L01_FLUSH_RX        0xE2

#define NRF24L01_TX_ADDR         0x10
#define NRF24L01_RX_ADDR_P0      0x0A

#define NRF24L01_CONFIG          0x00
#define NRF24L01_EN_AA           0x01
#define NRF24L01_EN_RXADDR       0x02
#define NRF24L01_SETUP_RETR      0x04
#define NRF24L01_RF_CH           0x05
#define NRF24L01_RF_SETUP        0x06
#define NRF24L01_STATUS          0x07

#define NRF24L01_MAX_TX  	0x10
#define NRF24L01_TX_OK   	0x20
#define NRF24L01_RX_OK   	0x40

#define NRF24L01_RX_PW_P0        0x11

#define TX_ADR_WIDTH    5
#define RX_ADR_WIDTH    5
#define TX_PLOAD_WIDTH  32
#define RX_PLOAD_WIDTH  32

uint8_t SPIx_ReadWriteByte(NRF24L01_t* nrf24l01,uint8_t byte)
{
  uint8_t d_read,d_send=byte;
  d_read = nrf24l01->opt->NRF24L01_TransmitReceiveByte(d_send);
  return d_read; 
}

uint8_t NRF24L01_Write_Buf(NRF24L01_t *nrf24l01, uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    uint8_t status, uint8_t_ctr;
    nrf24l01->opt->NRF24L01_CSN_L();
    status = SPIx_ReadWriteByte(nrf24l01, reg);
    for (uint8_t_ctr = 0; uint8_t_ctr < len; uint8_t_ctr++)
    {
        SPIx_ReadWriteByte(nrf24l01, *pBuf++);
    }
    nrf24l01->opt->NRF24L01_CSN_H();
    return status;
}

uint8_t NRF24L01_Read_Buf(NRF24L01_t *nrf24l01, uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    uint8_t status, uint8_t_ctr;
    nrf24l01->opt->NRF24L01_CSN_L();
    status = SPIx_ReadWriteByte(nrf24l01, reg);
    for (uint8_t_ctr = 0; uint8_t_ctr < len; uint8_t_ctr++)
    {
        pBuf[uint8_t_ctr] = SPIx_ReadWriteByte(nrf24l01, 0XFF);
    }
    nrf24l01->opt->NRF24L01_CSN_H();
    return status;
}

NRF24L01_Check_t NRF24L01_Check(NRF24L01_t* nrf24l01)
{
	uint8_t buf[5] = {0xA5, 0xA5, 0xA5, 0xA5, 0xA5};
	uint8_t i;
	NRF24L01_Write_Buf(nrf24l01,NRF24L01_WRITE_REG+NRF24L01_TX_ADDR,buf,5);
	NRF24L01_Read_Buf(nrf24l01,NRF24L01_TX_ADDR,buf,5);
	for(i=0;i<5;i++)if(buf[i]!=0XA5)break;	
	if(i!=5) return NRF24L01_CHECK_FALSE;
	return NRF24L01_CHECK_OK;
}

uint8_t NRF24L01_Write_Reg(NRF24L01_t* nrf24l01, uint8_t reg, uint8_t value)
{
	uint8_t status;
	nrf24l01->opt->NRF24L01_CSN_L();
	status =SPIx_ReadWriteByte(nrf24l01,reg);
	SPIx_ReadWriteByte(nrf24l01,value);
	nrf24l01->opt->NRF24L01_CSN_H();
	return status;
}

uint8_t NRF24L01_Read_Reg(NRF24L01_t* nrf24l01, uint8_t reg)
{
	uint8_t reg_val;
	nrf24l01->opt->NRF24L01_CSN_L();
	SPIx_ReadWriteByte(nrf24l01,reg);
	reg_val=SPIx_ReadWriteByte(nrf24l01,0XFF);
	nrf24l01->opt->NRF24L01_CSN_H();
	return reg_val;
}

uint8_t NRF24L01_TxPacket(NRF24L01_t* nrf24l01,uint8_t *txbuf)
{
    uint8_t sta;
	nrf24l01->opt->NRF24L01_CSN_L();
	NRF24L01_Write_Buf(nrf24l01,NRF24L01_WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);
	nrf24l01->opt->NRF24L01_CSN_H();
	
	while(nrf24l01->opt->NRF24L01_IRQ_R() != 0);
	
	sta = NRF24L01_Read_Reg(nrf24l01, NRF24L01_STATUS);
	NRF24L01_Write_Reg(nrf24l01, NRF24L01_WRITE_REG+NRF24L01_STATUS,sta);
	if(sta&NRF24L01_MAX_TX)
	{
		NRF24L01_Write_Reg(nrf24l01, NRF24L01_FLUSH_TX,0xff);
		return NRF24L01_MAX_TX;
	}
	if(sta&NRF24L01_TX_OK) return NRF24L01_TX_OK;
	return 0xFF;
}

uint8_t NRF24L01_RxPacket(NRF24L01_t* nrf24l01,uint8_t *rxbuf)
{
	uint8_t sta;
	sta = NRF24L01_Read_Reg(nrf24l01,NRF24L01_STATUS);
	NRF24L01_Write_Reg(nrf24l01,NRF24L01_WRITE_REG+NRF24L01_STATUS,sta);
	if(sta&NRF24L01_RX_OK)
	{
		NRF24L01_Read_Buf(nrf24l01,NRF24L01_RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);
		NRF24L01_Write_Reg(nrf24l01,NRF24L01_FLUSH_RX,0xff);
		return 0;
	}
	return sta;
}

void NRF24L01_TX_Mode(NRF24L01_t* nrf24l01)
{
    nrf24l01->opt->NRF24L01_CE_L();
    NRF24L01_Write_Buf(nrf24l01, NRF24L01_WRITE_REG + NRF24L01_TX_ADDR, nrf24l01->TX_Address, TX_ADR_WIDTH);
    NRF24L01_Write_Buf(nrf24l01, NRF24L01_WRITE_REG + NRF24L01_RX_ADDR_P0, nrf24l01->RX_Address, RX_ADR_WIDTH);

    NRF24L01_Write_Reg(nrf24l01, NRF24L01_WRITE_REG + NRF24L01_EN_AA, 0x01);
    NRF24L01_Write_Reg(nrf24l01, NRF24L01_WRITE_REG + NRF24L01_EN_RXADDR, 0x01);
    NRF24L01_Write_Reg(nrf24l01, NRF24L01_WRITE_REG + NRF24L01_SETUP_RETR, 0xff);
    NRF24L01_Write_Reg(nrf24l01, NRF24L01_WRITE_REG + NRF24L01_RF_CH, 40);
    NRF24L01_Write_Reg(nrf24l01, NRF24L01_WRITE_REG + NRF24L01_RF_SETUP, 0x0f);
    NRF24L01_Write_Reg(nrf24l01, NRF24L01_WRITE_REG + NRF24L01_CONFIG, 0x0e);

    nrf24l01->opt->NRF24L01_CE_H();
    nrf24l01->opt->NRF24L01_Delay(1);
}

void NRF24L01_RX_Mode(NRF24L01_t* nrf24l01)
{
    nrf24l01->opt->NRF24L01_CE_L();

    NRF24L01_Write_Reg(nrf24l01,NRF24L01_WRITE_REG+NRF24L01_CONFIG,0x0f);
	NRF24L01_Write_Reg(nrf24l01,NRF24L01_WRITE_REG+NRF24L01_EN_AA,0x01);
	NRF24L01_Write_Reg(nrf24l01,NRF24L01_WRITE_REG+NRF24L01_EN_RXADDR,0x01);
	NRF24L01_Write_Reg(nrf24l01,NRF24L01_WRITE_REG+NRF24L01_RF_CH,40);
	NRF24L01_Write_Reg(nrf24l01,NRF24L01_WRITE_REG+NRF24L01_RF_SETUP,0x0f);
	
	NRF24L01_Write_Reg(nrf24l01,NRF24L01_WRITE_REG+NRF24L01_RX_PW_P0,RX_PLOAD_WIDTH);

	NRF24L01_Write_Buf(nrf24l01,NRF24L01_WRITE_REG+NRF24L01_RX_ADDR_P0,nrf24l01->RX_Address,RX_ADR_WIDTH);

    nrf24l01->opt->NRF24L01_CE_H();
    nrf24l01->opt->NRF24L01_Delay(1);
}
