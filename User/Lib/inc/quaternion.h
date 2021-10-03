#ifndef __QUATERNION_H_
#define __QUATERNION_H_

#include "stm32f4xx.h"
#include "imu_mpu6050.h"
#include "math.h"
#include "math2.h"
#include "usart3.h"

void Imu_AHRS_Update(void);
void Init_Quaternion(void);
void Imu_Attitude_Update(void);

#endif //__QUATERNION_H_
