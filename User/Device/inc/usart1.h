/*
 * @Author: your name
 * @Date: 2021-09-27 13:12:35
 * @LastEditTime: 2021-09-27 19:03:59
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \MDK-ARMe:\Hero\User\Device\inc\usart1.h
 */
#ifndef __USART1_H_
#define __USART1_H_

#include "stm32f4xx_ll_usart.h"
#include "usart.h"
#include "usart3.h"
#include "remote_task.h"

void Usart1_DMA_RxCp_Callback(void);
void Usart1_RxDMA_Init(void);
uint8_t *Get_Usart1_DMA_RxBuffer_One(void);
uint8_t *Get_Usart1_DMA_RxBuffer_Two(void);
const uint16_t *Get_Usart1_DMA_RxMaxLen(void);
uint16_t *Get_Usart1_DMA_Rxd_DataLen(void);
uint8_t Get_Rc_Available_Bufferx(void);
void Usart1_Rx_DMA_Reset(void);
#endif //__USART1_H_
