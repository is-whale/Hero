#include "can1_device.h"

static Pid_Position_t motor_fl_speed_pid = NEW_POSITION_PID(11, 0, 2, 2000, 16000, 0, 1000, 500);
static Pid_Position_t motor_fr_speed_pid = NEW_POSITION_PID(11, 0, 2, 2000, 16000, 0, 1000, 500);
static Pid_Position_t motor_bl_speed_pid = NEW_POSITION_PID(11, 0, 2, 2000, 16000, 0, 1000, 500);
static Pid_Position_t motor_br_speed_pid = NEW_POSITION_PID(11, 0, 2, 2000, 16000, 0, 1000, 500);

void Set_ChassisMotor_Speed(float speed_fl, float speed_fr, float speed_bl, float speed_br, Motor_Measure_t *chassis_motor_feedback_data)
{
    int16_t a = Pid_Position_Calc(&motor_fl_speed_pid, speed_fl, chassis_motor_feedback_data[0].speed_rpm);
    int16_t b = Pid_Position_Calc(&motor_fr_speed_pid, speed_fr, chassis_motor_feedback_data[1].speed_rpm);
    int16_t c = Pid_Position_Calc(&motor_bl_speed_pid, speed_bl, chassis_motor_feedback_data[2].speed_rpm);
    int16_t d = Pid_Position_Calc(&motor_br_speed_pid, speed_br, chassis_motor_feedback_data[3].speed_rpm);
    Can1_Send_4Msg(
        CAN_CHASSIS_ALL_ID,
        a,
        b,
        c,
        d);
    // debug_log("%d %d %d %d\r\n", chassis_motor_feedback_data[0].speed_rpm,
    //           chassis_motor_feedback_data[0].speed_rpm,
    //           chassis_motor_feedback_data[0].speed_rpm,
    //           chassis_motor_feedback_data[0].speed_rpm);
}
