#ifndef __USART3_TASK_H_
#define __USART3_TASK_H_

#include "usart.h"

void Usart3_Tx_Init(void);
void Usart3_Tx_Transmit_Dma(uint32_t data_address, uint32_t len);
void Usart3_It_Tc_Callback(void);

#endif //  __USART3_TASK_H_
