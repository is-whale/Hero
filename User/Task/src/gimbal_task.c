#include "gimbal_task.h"

static Motor_Measure_t *pitch_motor_feedback_parsed_data;
static Motor_Measure_t *yaw_motor_feedback_parsed_data;
static CAN_RxHeaderTypeDef *can2_rx_header_pt;
static uint8_t *can2_rxd_data_buffer;

static Rc_Ctrl_t *rc_data_pt;
static Robot_control_data_t *robot_mode_data_pt;

static Motor_Measure_t pitch_motor_parsed_feedback_data;

void StartGimbalTask(void const *argument)
{
    float yaw_speed, pitch_speed;
    Can2_Filter_Init();
    pitch_motor_feedback_parsed_data = Get_Pitch_Motor_Feedback_Data();
    yaw_motor_feedback_parsed_data = Get_Yaw_Motor_Feedback_Data();
    can2_rx_header_pt = Get_CAN2_Rx_Header();
    can2_rxd_data_buffer = Get_CAN2_Rxd_Buffer();
    rc_data_pt = Get_Rc_Parsed_RemoteData_Pointer();
    robot_mode_data_pt = Get_Parsed_RobotMode_Pointer();

    ///< 仅测试阶段使用
    robot_mode_data_pt->mode.control_device = 0;
    robot_mode_data_pt->mode.rc_motion_mode = 1;

    (void)yaw_motor_feedback_parsed_data;   ///< 暂时为了避免警告 unused warning
    (void)pitch_motor_feedback_parsed_data; ///< 暂时为了避免警告 unused warning

    osDelay(1000);
    __OPEN_CAN2_RX_FIFO0_IT__;

    for (;;)
    {
        // TODO 随后进行更改，改函数应解析所有的 CAN2 上云台电机的数据，此时最后一个参数这里只是为了测试
        Parse_Can2_Gimbal_Rxd_Data(can2_rx_header_pt, can2_rxd_data_buffer, &pitch_motor_parsed_feedback_data);
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

        Set_Gimbal_Motors_Speed(yaw_speed, pitch_speed, &pitch_motor_parsed_feedback_data, &pitch_motor_parsed_feedback_data);
        osDelay(10);
    }
}

void Parse_Can2_Gimbal_Rxd_Data(CAN_RxHeaderTypeDef *p_can_rx_header, uint8_t *data, Motor_Measure_t *motor)
{

    switch (p_can_rx_header->StdId)
    {
    case CAN_YAW_MOTOR_ID:
    {
        Calculate_Motor_Data(motor, data);
        break;
    }

    case CAN_PITCH_MOTOR_ID:
    {
        Calculate_Motor_Data(motor, data);
        break;
    }

    case 0:
    {
        // TODO 其他 CAN2 总线上的数据反馈解析，
        // 如果不在云台任务中，应当在 can2_device.c 中进行解析，或者在其他任务中解析
        break;
    }
    default:
    {
        break;
    }
    }
}
