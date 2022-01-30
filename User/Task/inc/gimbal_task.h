#ifndef __GIMBAL_TASK_H_
#define __GIMBAL_TASK_H_

#include "cmsis_os.h"
#include "can2.h"
#include "can2_device.h"
#include "remote_task.h"
#include "imu_mpu6050.h"
#include "console_print.h"

/*yaw*/
#define YAW_GM6020_HEAD_ANGLE 5450
/*pitch*/
#define PITCH_UP_LIMIT 6960///<pitch上限位
#define PITCH_MID_ANGLE 7490///<pitch中间附近
#define PITCH_DOWN_LIMIT 8010///<pitch下限位
//测量极限位置的电机角度返回值得到这些数据。如云台最高&最低能到达的位置

#define __OPEN_CAN2_RX_FIFO0_IT__ HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING); 

void Parse_Can2_Gimbal_Rxd_Data(CAN_RxHeaderTypeDef *p_can_rx_header, uint8_t data[], Motor_Measure_t *motor);
Motor_Measure_t *Get_Gimbal_Parsed_FeedBack_Data(void);
const uint8_t* Get_Pitch_Motor_Index(void);
const uint8_t* Get_Yaw_Motor_Index(void);
float GM6020_YAW_Angle_To_360(uint16_t gm6020_angle);

#endif //__GIMBAL_TASK_H_
