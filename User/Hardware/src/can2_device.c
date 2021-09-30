#include "can2_device.h"

static Motor_Measure_t gm6020_feedback_data[2];
static uint8_t *can2_rxd_data_buffer;
static const uint8_t yaw_motor_index = 0;
static const uint8_t pitch_motor_index = 1;
static CAN_RxHeaderTypeDef *can2_rx_header;

void Can2_Rx_FIFO0_IT_Callback(void)
{
    HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, can2_rx_header, can2_rxd_data_buffer);
    switch (can2_rx_header->StdId)
    {
    case CAN_YAW_MOTOR_ID:
    {
        Calculate_Motor_Data(&gm6020_feedback_data[yaw_motor_index], can2_rxd_data_buffer);
        break;
    }
    case CAN_PITCH_MOTOR_ID:
    {
        Calculate_Motor_Data(&gm6020_feedback_data[pitch_motor_index], can2_rxd_data_buffer);
        break;
    }
    default:
    {
        break;
    }
    }
}

/**
 * @brief                       返回解析后 pitch 轴的 gm6020 电机的数据
 * @param                       void
 * @return {Motor_Measure_t*}   指向 pitch 轴电机结构题数据的指针
 */
Motor_Measure_t *Get_Pitch_Motor_Feedback_Data(void)
{
    return &gm6020_feedback_data[yaw_motor_index];
}

/**
 * @brief                       返回解析后 yaw 轴的 gm6020 电机的数据
 * @param                       void
 * @return {Motor_Measure_t*}   指向 yaw 轴电机结构题数据的指针
 */
Motor_Measure_t *Get_Yaw_Motor_Feedback_Data(void)
{
    return &gm6020_feedback_data[pitch_motor_index];
}
