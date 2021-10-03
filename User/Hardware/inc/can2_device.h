#ifndef __CAN2_DEVICE_H_
#define __CAN2_DEVICE_H_

#include "can2.h"
#include "motor.h"
#include "pid.h"
#include "math2.h"
#include "usart3.h"

typedef enum
{
	CAN_SHOOTER_ALL_ID = 0x200,
	CAN_3508_WAVE_ID_LEFT = 0x201,
	CAN_3508_WAVE_ID_RIGHT = 0x202,
	
	CAN_GIMBAL_ALL_ID = 0x2FF,
	CAN_YAW_MOTOR_ID = 0x209,	///< x
	CAN_PITCH_MOTOR_ID = 0x20B, ///< y
} Can2_RxMsg_ID;

void Can2_Rx_FIFO0_IT_Callback(void);
Motor_Measure_t *Get_Pitch_Motor_Feedback_Data(void);
Motor_Measure_t *Get_Yaw_Motor_Feedback_Data(void);
const uint8_t *Get_Pitch_Motor_Index(void);
const uint8_t *Get_Yaw_Motor_Index(void);
Motor_Measure_t *Get_Gimbal_Motor_Feedback_Data(void);
CAN_RxHeaderTypeDef *Get_CAN2_Rx_Header(void);
CAN_RxHeaderTypeDef *Get_CAN2_Rx_Header(void);
uint8_t *Get_CAN2_Rxd_Buffer(void);
void Set_Gimbal_Motors_Speed(float yaw_speed, float pitch_speed, Motor_Measure_t *yaw_motor_parsed_feedback_data, Motor_Measure_t *pitch_motor_parsed_feedback_data);
void Pitch_Angle_Limit(float* angle, float down_angle, float up_angle);
float Calc_Pitch_Angle8191_Pid(float tar_angle,Motor_Measure_t *pitch_motor_parsed_feedback_data);
void Set_Wave_Motor_Speed(float speed_left, float speed_right, Motor_Measure_t *wave_motor_feedback_data);

#endif //__CAN2_DEVICE_H_
