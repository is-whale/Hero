#ifndef __CAN2_DEVICE_H_
#define __CAN2_DEVICE_H_

#include "can2.h"
#include "motor.h"
#include "pid.h"

typedef enum
{
	CAN_SHOOTER_ALL_ID = 0x200,
	CAN_3508_WAVE_ID = 0x201,

	CAN_GIMBAL_ALL_ID = 0x2FF,
	CAN_YAW_MOTOR_ID = 0x209,       ///< x
	CAN_PITCH_MOTOR_ID = 0x20A,     ///< y
} Can2_RxMsg_ID;

void Can2_Rx_FIFO0_IT_Callback(void);
Motor_Measure_t *Get_Pitch_Motor_Feedback_Data(void);
Motor_Measure_t *Get_Pitch_Motor_Feedback_Data(void);

#endif //__CAN2_DEVICE_H_
