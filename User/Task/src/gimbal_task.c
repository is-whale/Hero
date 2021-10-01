#include "gimbal_task.h"

static const uint8_t gimbal_motor_num = 2;  ///< ��̨���������
static const uint8_t yaw_motor_index = 0;   ///< yaw �����ڵ�����ݽṹ���е��±�
static const uint8_t pitch_motor_index = 1; ///< pitch �����ڵ�����ݽṹ���е��±�

static CAN_RxHeaderTypeDef *can2_rx_header_pt;   ///< can2 ���յ�ͷ���ݽṹ��ָ��
static uint8_t *can2_rxd_data_buffer;            ///< can2 ���յ����ݴ�ŵ������׵�ַ
static Rc_Ctrl_t *rc_data_pt;                    ///< ָ��ң�������ݵĽṹ��ָ��
static Robot_control_data_t *robot_mode_data_pt; ///< ָ�������ģʽ�Ľṹ��ָ��

static Motor_Measure_t gimbal_motor_parsed_feedback_data[gimbal_motor_num]; ///< ���������̨�����������

void StartGimbalTask(void const *argument)
{
    float yaw_speed, pitch_speed;
    Can2_Filter_Init();
    can2_rx_header_pt = Get_CAN2_Rx_Header();
    can2_rxd_data_buffer = Get_CAN2_Rxd_Buffer();
    rc_data_pt = Get_Rc_Parsed_RemoteData_Pointer();
    robot_mode_data_pt = Get_Parsed_RobotMode_Pointer();

    ///< �����Խ׶�ʹ��
    robot_mode_data_pt->mode.control_device = 0;
    robot_mode_data_pt->mode.rc_motion_mode = 1;

    osDelay(1000);
    __OPEN_CAN2_RX_FIFO0_IT__;

    for (;;)
    {
        Parse_Can2_Gimbal_Rxd_Data(can2_rx_header_pt, can2_rxd_data_buffer, gimbal_motor_parsed_feedback_data);
        if (robot_mode_data_pt->mode.control_device == 0)
        {
            switch (1)
            {
            case 1:

            {
                yaw_speed = (-rc_data_pt->rc.ch0) * 0.5f;
                pitch_speed = (rc_data_pt->rc.ch1) * 0.5f;
                break;
            }

            default:
            {
                break;
            }
            }
        }

        Set_Gimbal_Motors_Speed(yaw_speed, pitch_speed, &gimbal_motor_parsed_feedback_data[yaw_motor_index], &gimbal_motor_parsed_feedback_data[pitch_motor_index]);
        osDelay(10);
    }
}

/**
 * @brief                       ���� CAN2 ������̨���յ���������̨�����ݲ����н���
 * @param p_can_rx_header       ָ����� CAN2 ����ͷ�Ľṹ��ָ��
 * @param data                  ���յ��� 8 Byte ����
 * @param motor                 �������ĵ���ṹ�������׵�ַ
 * @retval                      void
 */
void Parse_Can2_Gimbal_Rxd_Data(CAN_RxHeaderTypeDef *p_can_rx_header, uint8_t *data, Motor_Measure_t *motor)
{

    switch (p_can_rx_header->StdId)
    {
    case CAN_YAW_MOTOR_ID:
    {
        Calculate_Motor_Data(&gimbal_motor_parsed_feedback_data[yaw_motor_index], data);
        break;
    }

    case CAN_PITCH_MOTOR_ID:
    {
        Calculate_Motor_Data(&gimbal_motor_parsed_feedback_data[pitch_motor_index], data);
        break;
    }

    case 0:
    {
        // TODO ���� CAN2 �����ϵ����ݷ���������
        // ���������̨�����У�Ӧ���� can2_device.c �н��н��������������������н���
        break;
    }
    default:
    {
        break;
    }
    }
}
