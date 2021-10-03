#ifndef __SHOOT_TASK_H_
#define __SHOOT_TASK_H_

#include "cmsis_os.h"
#include "stm32f4xx.h"
#include "remote_task.h"
#include "can2_device.h"

void Parse_Shoot_Wave_Motor_Feedback_Data(CAN_RxHeaderTypeDef *p_can_rx_header,uint8_t *can2_rxd_data_buffer);

#endif //__SHOOT_TASK_H_
