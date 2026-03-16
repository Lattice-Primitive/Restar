/**
 * @file drv_nrf24l01
 * @author
 * @note nrf24l01无线2.4G模块，调不通
*/
#ifndef __DRV_NRF24L01_H
#define __DRV_NRF24L01_H

#ifdef __cplusplus
extern "C"
{
#endif /*__cplusplus*/

#include <stdint.h>

typedef enum
{
    NRF24L01_CHECK_OK=0,
    NRF24L01_CHECK_FALSE=1,
}NRF24L01_Check_t;

typedef struct
{
    uint8_t (*NRF24L01_TransmitReceiveByte)(uint8_t txdata);
    void (*NRF24L01_CE_H)(void);
    void (*NRF24L01_CE_L)(void);
    void (*NRF24L01_CSN_H)(void);
    void (*NRF24L01_CSN_L)(void);
    uint32_t (*NRF24L01_IRQ_R)(void);
    void (*NRF24L01_Delay)(uint32_t ticks);
}NRF24L01_Operate_t;

typedef struct
{
    uint8_t *TX_Address;
    uint8_t *RX_Address;
    NRF24L01_Operate_t* opt;
}NRF24L01_t;

extern NRF24L01_Check_t NRF24L01_Check(NRF24L01_t* nrf24l01);

extern void NRF24L01_TX_Mode(NRF24L01_t* nrf24l01);

extern void NRF24L01_RX_Mode(NRF24L01_t* nrf24l01);

extern uint8_t NRF24L01_TxPacket(NRF24L01_t* nrf24l01,uint8_t *txbuf);

extern uint8_t NRF24L01_RxPacket(NRF24L01_t* nrf24l01,uint8_t *rxbuf);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif
