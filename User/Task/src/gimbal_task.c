#include "gimbal_task.h"

static Motor_Measure_t *pitch_motor_feedback_parsed_data;
static Motor_Measure_t *yaw_motor_feedback_parsed_data;
static CAN_RxHeaderTypeDef *can2_rx_header_pt;

void StartGimbalTask(void const *argument)
{

    Can2_Filter_Init();
    pitch_motor_feedback_parsed_data = Get_Pitch_Motor_Feedback_Data();
    yaw_motor_feedback_parsed_data = Get_Yaw_Motor_Feedback_Data();
    can2_rx_header_pt = Get_CAN2_Rx_Header();
    (void)yaw_motor_feedback_parsed_data; ///< 暂时为了避免警告 unused warning
    (void)can2_rx_header_pt;              ///< 暂时为了避免警告 unused warning,供后面测试使用
    
    osDelay(1000);
    __OPEN_CAN2_RX_FIFO0_IT__;

    for (;;)
    {

        // debug_print("angle :%d speed %d\r\n", pitch_motor_feedback_parsed_data->mechanical_angle, pitch_motor_feedback_parsed_data->speed_rpm);

        osDelay(5);
    }
}
