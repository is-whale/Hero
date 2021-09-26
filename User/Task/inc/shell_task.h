#ifndef __SHELL_TASK_
#define __SHELL_TASK_

#include "cmsis_os.h"
#include "stm32f4xx.h"
#include "usart.h"
#include "stdio.h"
#include "usart3.h"
#include "stm32f4xx_ll_usart.h."
#include "usart3.h"

#define PRINT_USART_INIT() Usart3_Rx_Init()

void Usart3_Idle_ITCallback(void);
void Usart_Send_Byte(USART_TypeDef *pUsart, uint8_t ch);
void Usart_Send_String(USART_TypeDef *pUsart,char *str);
void Usart_Send_HalfWord(USART_TypeDef *pUsart, uint16_t ch);
void Usart_Send_ArrayBuffer(USART_TypeDef *pUsart, uint8_t *array, uint16_t length);

#endif // __SHELL_TASK_
