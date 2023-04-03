/**
 * @file gyroscope_task.c
 * @brief ���������ǽ�������
 * TODO: Kalman Filter
 * @version 0.1
 * @date 2021-10-17
 * @copyright Copyright (c) 2021
 */
/* ��ʹ���������������ݣ�������ʱע�����������������ݣ���������Ҫ��ɾ���������� */

#include "gyroscope_task.h"
#include "quaternion.h"

void StartGyroscopeTask(void const *argument)
{
  osDelay(100);

  Mpu_Device_Init();
  Init_Quaternion();

  for (;;)
  {
    //��ʼ������������
    /*Mpu6050_Get_Data();
    Imu_AHRS_Update();

    Imu_Attitude_Update(); */
    
    ///<����Ƿ���Ҫ���ݼ�麯��
    //  imu_date = Get_imu_date_now;

    osDelay(1);
  }
}
