#ifndef __GIMBAL_TASK_H_
#define __GIMBAL_TASK_H_

#include "cmsis_os.h"
#include "can2.h"
#include "can2_device.h"
#include "remote_task.h"

#define __OPEN_CAN2_RX_FIFO0_IT__ HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING); 
void Parse_Can2_Gimbal_Rxd_Data(CAN_RxHeaderTypeDef *p_can_rx_header, uint8_t data[], Motor_Measure_t *motor);

#endif //__GIMBAL_TASK_H_
