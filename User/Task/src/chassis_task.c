#include "chassis_task.h"

static Rc_Ctrl_t *rc_data_pt;
static Robot_control_data_t *robot_mode_data_pt;
static const float motor_speed_multiple = 13.5;
static Motor_Measure_t *chassis_motor_feedback_parsed_data;

void StartChassisTask(void const *argument)
{
    static float chassis_motor_speed[4] = {0.0, 0.0, 0.0, 0.0};
    rc_data_pt = Get_Rc_Parsed_RemoteData_Pointer();
    robot_mode_data_pt = Get_Parsed_RobotMode_Pointer();
    chassis_motor_feedback_parsed_data = Get_Can1_Feedback_Data();

    ///< 仅测试阶段使用
    robot_mode_data_pt->mode.control_device = 0;
    robot_mode_data_pt->mode.rc_motion_mode = 1;
    osDelay(100);

    for (;;)
    {
        // debug_print("%d, %d, %d, %d\r\n", robot_mode_data_pt->virtual_rocker.ch0,
        //             robot_mode_data_pt->virtual_rocker.ch1,
        //             robot_mode_data_pt->virtual_rocker.ch2,
        //             robot_mode_data_pt->virtual_rocker.ch3);

        chassis_motor_speed[0] = -robot_mode_data_pt->virtual_rocker.ch3 - robot_mode_data_pt->virtual_rocker.ch2 + robot_mode_data_pt->virtual_rocker.ch0;
        chassis_motor_speed[1] = robot_mode_data_pt->virtual_rocker.ch3- robot_mode_data_pt->virtual_rocker.ch2 + robot_mode_data_pt->virtual_rocker.ch0;
        chassis_motor_speed[2] = -robot_mode_data_pt->virtual_rocker.ch3 + robot_mode_data_pt->virtual_rocker.ch2 + robot_mode_data_pt->virtual_rocker.ch0;
        chassis_motor_speed[3] = robot_mode_data_pt->virtual_rocker.ch3 + robot_mode_data_pt->virtual_rocker.ch2 + robot_mode_data_pt->virtual_rocker.ch0;

        chassis_motor_speed[0] *=1.0f;
        chassis_motor_speed[1] *=1.0f;
        chassis_motor_speed[2] *=1.0f;
        chassis_motor_speed[3] *=1.0f;

        if (robot_mode_data_pt->mode.control_device == 2) ///< 最好是使用枚举定义
        {
            switch (robot_mode_data_pt->mode.rc_motion_mode)
            {
            case 1: ///< 改用枚举类型
            {
                chassis_motor_speed[0] = -rc_data_pt->rc.ch3 - rc_data_pt->rc.ch2 + rc_data_pt->rc.ch0;
                chassis_motor_speed[1] = rc_data_pt->rc.ch3 - rc_data_pt->rc.ch2 + rc_data_pt->rc.ch0;
                chassis_motor_speed[2] = -rc_data_pt->rc.ch3 + rc_data_pt->rc.ch2 + rc_data_pt->rc.ch0;
                chassis_motor_speed[3] = rc_data_pt->rc.ch3 + rc_data_pt->rc.ch2 + rc_data_pt->rc.ch0;

                chassis_motor_speed[0] *= motor_speed_multiple;
                chassis_motor_speed[1] *= motor_speed_multiple;
                chassis_motor_speed[2] *= motor_speed_multiple;
                chassis_motor_speed[3] *= motor_speed_multiple;

                break;
            }

            default:
            {
                break;
            }
            }
        }
        else if (robot_mode_data_pt->mode.control_device == 2)
        {
            switch (robot_mode_data_pt->mode.mouse_keyboard_chassis_mode)
            {
            case 1:

            {
                break;
            }

            default:

            {
                break;
            }
            }
        }

        ///< 设置底盘电机速度
        Set_ChassisMotor_Speed(chassis_motor_speed[0], chassis_motor_speed[1], chassis_motor_speed[2], chassis_motor_speed[3], chassis_motor_feedback_parsed_data);

        osDelay(10);
    }
}
