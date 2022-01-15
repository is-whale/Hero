/**
 * @file gyroscope_task.c
 * @brief 板载陀螺仪解析任务
 * TODO: Kalman Filter
 * @version 0.1
 * @date 2021-10-17 
 * @copyright Copyright (c) 2021
 */

/*inlcude*/
#include "gyroscope_task.h"
#include "quaternion.h"
// static Imu_t*imu_date;
void StartGyroscopeTask(void const * argument)
{
    osDelay(100);

    Mpu_Device_Init();
    Init_Quaternion();
    
    for(;;)
    {
        //初始化及更新数据
        Mpu6050_Get_Data();
        Imu_AHRS_Update();

        Imu_Attitude_Update();///<检查是否需要数据检查函数
      //  imu_date = Get_imu_date_now;
        
        osDelay(1);
    }
}
