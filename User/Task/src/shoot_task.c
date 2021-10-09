/**
 * @file shoot_task.c
 * @brief ����������������� 3508 ����� id �ֱ�Ϊ 201 �� 202, ������ can2 ������
 *        ����������ݷֱ���� shoot_motor_feedback_data[0]��shoot_motor_feedback_data[1]
 * @version 0.1
 * @date 2021-10-02
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "shoot_task.h"

static Rc_Ctrl_t *rc_data_pt;
static Robot_control_data_t *robot_control_data_pt;
static float fric_speed = 0;
static float wave_motor_speed = 0.0;
static CAN_RxHeaderTypeDef *can2_rx_header_p;
static uint8_t *can2_rxd_data_buffer;
static const uint8_t friction_motor_num = 2;
static Motor_Measure_t friction_motor_feedback_data[friction_motor_num];
static Motor_Measure_t *wave_motor_feedback_data;
static int *erroe_integral;
static uint16_t *last_machine_angle;
static uint16_t *this_machine_angle;

void StartShootTask(void const *argument)
{
    rc_data_pt = Get_Rc_Parsed_RemoteData_Pointer();
    robot_control_data_pt = Get_Parsed_RobotMode_Pointer();
    can2_rxd_data_buffer = Get_CAN2_Rxd_Buffer();
    can2_rx_header_p = Get_CAN2_Rx_Header();
    wave_motor_feedback_data = Get_Wave_Motor_Paresed_Data();

    erroe_integral = Get_Error_Integral();
    last_machine_angle = Get_Last_Machine_Angle();
    this_machine_angle = Get_This_Machine_Angle();

    (void)rc_data_pt; ///< ���� warning

    osDelay(100);

    for (;;)
    {
        Parse_Friction_Wave_Motor_Feedback_Data(can2_rx_header_p, can2_rxd_data_buffer);
        switch (robot_control_data_pt->mode.fric_cover_mode)
        {
        case fric_cover_off_mode_ENUM: ///< 0

        {
            fric_speed = 0;
            static uint32_t wave_once_machine_angle = 120000;
            // wave_motor_speed = rc_data_pt->rc.ch1;
						//debug_print("inte %d speed %d \r\n",*erroe_integral,wave_motor_feedback_data->speed_rpm);
            wave_motor_speed = Calc_Wave_Motor_Angle8191_Pid(wave_once_machine_angle, *erroe_integral);

            break;
        }

        case fric_adaptive_speed_mode_ENUM: ///< 1

        {
            fric_speed = 5000;
            wave_motor_speed = 5000;
            /* fric_speed = rc_data_pt->rc.ch1;
            fric_speed *= 5; */
            break;
        }

        case fric_high_speed_mode_ENUM: ///< 2

        {
            fric_speed = 7000;
            wave_motor_speed = 7000;
            /*  fric_speed = rc_data_pt->rc.ch1;
            fric_speed *= 5; */
            break;
        }

        case cover_on_ENUM: ///< 3

        {
            fric_speed = 10000;
            wave_motor_speed = 10000;
            /* fric_speed = rc_data_pt->rc.ch1;
            fric_speed *= 5; */
            break;
        }

        default:
        {
            break;
        }
        }

        Set_Friction_Motor_Speed(fric_speed, fric_speed, friction_motor_feedback_data);
        Set_Wave_Motor_Speed(wave_motor_speed, wave_motor_feedback_data);
        debug_print("speed: %d, angle: %d : angle %d\r\n", wave_motor_feedback_data->speed_rpm, wave_motor_feedback_data->mechanical_angle,*erroe_integral);
        osDelay(10);
    }
}

/**
 * @brief                           ������̨����Ħ���ֵ���Ͳ��ֵ��������
 * @param CAN_RxHeaderTypeDef       can2 ���ܽṹ��ͷָ��
 * @param uint8_t_*                 can2 ���ܵ�������
 * @retval                          void
 */
void Parse_Friction_Wave_Motor_Feedback_Data(CAN_RxHeaderTypeDef *can_rx_header, uint8_t *can2_rxd_data_buffer)
{
    static uint8_t i = 0;
    switch (can_rx_header->StdId)
    {
    case CAN_3508_FRIC_ID_LEFT:
    case CAN_3508_FRIC_ID_RIGHT:
    {
        i = can_rx_header->StdId - CAN_SHOOTER_ALL_ID - 1;
        Calculate_Motor_Data(&friction_motor_feedback_data[i], can2_rxd_data_buffer);
        break;
    }

    default:
        break;
    }
}
