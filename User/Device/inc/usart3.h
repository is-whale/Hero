#ifndef __USART3_H_
#define __USART3_H_

#include "stm32f4xx.h"
#include "usart.h"
#include "stdio.h"
void Usart3_Tx_Init(void);
void Usart3_Transmit_Dma(uint32_t data_address, uint32_t len);
void Usart3_It_Tc_Callback(void);

void Usart3_Rx_Init(void);
uint8_t *Get_Usart3_DMA_RxBuffer(void);
const uint16_t *Get_Usart3_DMA_Rx_MaxLen(void);

#endif //__USART3_H_

