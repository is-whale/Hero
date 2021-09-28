#ifndef _CAN1_H_
#define _CAN1_H_

#include "cmsis_os.h"
#include "stdio.h"
#include "stm32f4xx.h"
#include "can.h"
#include "parse_can1_rx_data_task.h"

void Can1_Send_4Msg(uint32_t id, int16_t data1, int16_t data2, int16_t data3, int16_t data4);
void Can1_Filter_Init(void);
void Can1_Rx_FIFO0_IT_Callback(void);

#endif //_CAN1_H_
