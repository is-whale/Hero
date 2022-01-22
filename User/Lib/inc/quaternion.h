#ifndef __QUATERNION_H_
#define __QUATERNION_H_

#include "stm32f4xx.h"
#include "imu_mpu6050.h"
#include "math.h"
#include "math2.h"
#include "usart3.h"
#include "console_print.h"
//typedef struct
//{
//	float rol_angle = 0;
//	float pit_angle = 0;
//	float yaw_angle = 0;
//} Imu_Angle;///<��Ž����ĽǶ�

void Imu_AHRS_Update(void);
void Init_Quaternion(void);///<��ʼ����Ԫ��
void Imu_Attitude_Update(void);//��������������
#endif //__QUATERNION_H_
