#ifndef __QUATERNION_H_
#define __QUATERNION_H_

#include "stm32f4xx.h"
#include "imu_mpu6050.h"
#include "math.h"
#include "math2.h"
#include "usart3.h"

//typedef struct
//{
//	float rol_angle = 0;
//	float pit_angle = 0;
//	float yaw_angle = 0;
//} Imu_Angle;///<存放结算后的角度

void Imu_AHRS_Update(void);
void Init_Quaternion(void);
void Imu_Attitude_Update(void);
//const void Get_Imu_Angle(Imu_Angle*)
#endif //__QUATERNION_H_
