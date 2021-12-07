/**
 * @file gyroscope_task.c
 * @brief ∞Â‘ÿÕ”¬›“«Ω‚Œˆ»ŒŒÒ
 * TODO: Kalman Filter
 * @version 0.1
 * @date 2021-10-17
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "gyroscope_task.h"
static Imu_t * imu_date_pt;///?????IMU??
extern imu;
void StartGyroscopeTask(void const * argument)
{
    osDelay(100);

    Mpu_Device_Init();
    Init_Quaternion();

    for(;;)
    {
        Mpu6050_Get_Data();
        Imu_AHRS_Update();
        Imu_Attitude_Update();
        Imu_t *Get_IMU_Date(void)
        {
            return &imu;
        }

        osDelay(100);
    }
}
