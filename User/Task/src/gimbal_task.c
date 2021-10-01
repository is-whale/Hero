#include "gimbal_task.h"

static const uint8_t gimbal_motor_num = 2;  ///< 云台电机的数量
static const uint8_t yaw_motor_index = 0;   ///< yaw 轴电机在电机数据结构体中的下标
static const uint8_t pitch_motor_index = 1; ///< pitch 轴电机在电机数据结构体中的下标

static CAN_RxHeaderTypeDef *can2_rx_header_pt;   ///< can2 接收的头数据结构体指针
static uint8_t *can2_rxd_data_buffer;            ///< can2 接收的数据存放的数组首地址
static Rc_Ctrl_t *rc_data_pt;                    ///< 指向遥控器数据的结构体指针
static Robot_control_data_t *robot_mode_data_pt; ///< 指向机器人模式的结构体指针

static Motor_Measure_t gimbal_motor_parsed_feedback_data[gimbal_motor_num]; ///< 解析后的云台电机数据数组

void StartGimbalTask(void const *argument)
{
    float yaw_speed, pitch_speed;
    Can2_Filter_Init();
    can2_rx_header_pt = Get_CAN2_Rx_Header();
    can2_rxd_data_buffer = Get_CAN2_Rxd_Buffer();
    rc_data_pt = Get_Rc_Parsed_RemoteData_Pointer();
    robot_mode_data_pt = Get_Parsed_RobotMode_Pointer();

    ///< 仅测试阶段使用
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
 * @brief                       解析 CAN2 解析云台接收到的两个云台的数据并进行解析
 * @param p_can_rx_header       指向接收 CAN2 数据头的结构体指针
 * @param data                  接收到的 8 Byte 数据
 * @param motor                 传进来的电机结构体数组首地址
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
