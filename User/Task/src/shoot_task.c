/**
 * @file shoot_task.c
 * @brief 发射机构，两个波轮 3508 电机的 id 分别为 201 和 202, 挂载在 can2 总线上
 *        解析后的数据分别放在 shoot_motor_feedback_data[0]、shoot_motor_feedback_data[1]
 * @version 0.1
 * @date 2021-10-02
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "shoot_task.h"

static Rc_Ctrl_t *rc_data_pt;
static Robot_control_data_t *robot_control_data_pt;
static float wave_speed = 0;
static CAN_RxHeaderTypeDef *can2_rx_header_p;
static uint8_t *can2_rxd_data_buffer;
static const uint8_t shoot_motor_num = 2;
static Motor_Measure_t shoot_motor_feedback_data[shoot_motor_num];

void StartShootTask(void const *argument)
{
    rc_data_pt = Get_Rc_Parsed_RemoteData_Pointer();
    robot_control_data_pt = Get_Parsed_RobotMode_Pointer();
    can2_rxd_data_buffer = Get_CAN2_Rxd_Buffer();
    can2_rx_header_p = Get_CAN2_Rx_Header();

    osDelay(100);

    for (;;)
    {
        Parse_Shoot_Wave_Motor_Feedback_Data(can2_rx_header_p, can2_rxd_data_buffer);
        switch (robot_control_data_pt->mode.fric_cover_mode)
        {
        case fric_cover_off_mode_ENUM:

        {
            wave_speed = 0;
            break;
        }

        case fric_adaptive_speed_mode_ENUM:

        {
            wave_speed = 5000;
            /* wave_speed = rc_data_pt->rc.ch1;
            wave_speed *= 5; */
            break;
        }

        case fric_high_speed_mode_ENUM:

        {
            wave_speed = 7000;
            /*  wave_speed = rc_data_pt->rc.ch1;
            wave_speed *= 5; */
            break;
        }

        case cover_on_ENUM:

        {
            wave_speed = 10000;
            /* wave_speed = rc_data_pt->rc.ch1;
            wave_speed *= 5; */
            break;
        }

        default:
        {
            break;
        }
        }
        Set_Wave_Motor_Speed(wave_speed, wave_speed, shoot_motor_feedback_data);
        osDelay(10);
    }
}

void Parse_Shoot_Wave_Motor_Feedback_Data(CAN_RxHeaderTypeDef *can_rx_header, uint8_t *can2_rxd_data_buffer)
{
    static uint8_t i = 0;
    switch (can_rx_header->StdId)
    {
    case CAN_3508_WAVE_ID_LEFT:
    case CAN_3508_WAVE_ID_RIGHT:
    {
        i = can_rx_header->StdId - CAN_SHOOTER_ALL_ID - 1;
        Calculate_Motor_Data(&shoot_motor_feedback_data[i], can2_rxd_data_buffer);
        break;
    }

    default:
        break;
    }
}
