/*
 * @Author: your name
 * @Date: 2021-09-26 14:07:44
 * @LastEditTime: 2021-09-26 22:22:35
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \MDK-ARMc:\Document\Workspace\Hero2022\User\Device\inc\usart3.h
 */
#ifndef __USART3_TASK_H_
#define __USART3_TASK_H_

#include <stdarg.h>
#include "stdio.h"
#include "usart.h"

void Usart3_Tx_Init(void);
void Usart3_Transmit_Dma(uint32_t data_address, uint32_t len);
void Usart3_It_Tc_Callback(void);
int __printf(const char *format, ...);

#endif //  __USART3_TASK_H_
