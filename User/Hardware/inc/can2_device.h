#ifndef __CAN2_DEVICE_H_
#define __CAN2_DEVICE_H_

#include "can2.h"
#include "motor.h"
#include "pid.h"
#include "math2.h"
#include "math.h"
#include "stdio.h"
#include "usart3.h"

#define WAVE_ONCE_BULLET 20000

typedef enum
{
	CAN_SHOOTER_ALL_ID = 0x200,
	CAN_3508_FRIC_ID_LEFT = 0x201,
	CAN_3508_FRIC_ID_RIGHT = 0x202,
	CAN_3508_WAVE_ID = 0x206,///<波轮ID从6更换为3
	
	CAN_GIMBAL_ALL_ID = 0x2FF,
	CAN_YAW_MOTOR_ID = 0x209,	///< x  5
	CAN_PITCH_MOTOR_ID = 0x20A, ///< y  
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
float Calc_Yaw_Angle360_Pid(float tar_angle, float cur_angle);
void Set_Friction_Motor_Speed(float speed_left, float speed_right, Motor_Measure_t *friction_motor_feedback_data);
void Set_Wave_Motor_Speed(float wave_motor_speed, Motor_Measure_t *wave_motor_feedback_data);
Motor_Measure_t *Get_Wave_Motor_Paresed_Data(void);
float Calc_Wave_Motor_Angle8191_Pid(float tar_angle, float current_angle);
int *Get_Error_Integral(void);
uint16_t *Get_Last_Machine_Angle(void);
uint16_t *Get_This_Machine_Angle(void);

#endif //__CAN2_DEVICE_H_
