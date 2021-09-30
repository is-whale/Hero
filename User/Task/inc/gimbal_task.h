#ifndef __GIMBAL_TASK_H_
#define __GIMBAL_TASK_H_

#include "cmsis_os.h"
#include "can2.h"
#include "can2_device.h"
#include "usart3.h"

#define __OPEN_CAN2_RX_FIFO0_IT__ HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING); 

#endif //__GIMBAL_TASK_H_
