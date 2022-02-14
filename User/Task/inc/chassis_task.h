#ifndef __CHASSIS_TASK_H_
#define __CHASSIS_TASK_H_

#include "remote_task.h"
#include "can1_device.h"
#include "gimbal_task.h"
#include "math.h"
#include "math2.h"
// #include "referee_system_task.h"
#include "referee_system.h"

#define OUTPUT_LIMIT(data, limit) Float_Constraion(data, -limit, limit)

void Calc_Gyro_Motors_Speed(float *motors_speed, float rotate_speed, float move_direction, float x_move_speed, float y_move_speed);
float Calc_Chassis_Follow(void);

#endif //__CHASSIS_TASK_H_
