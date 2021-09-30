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
 * @brief                       ���ؽ����� pitch ��� gm6020 ���������
 * @param                       void
 * @return {Motor_Measure_t*}   ָ�� pitch �����ṹ�����ݵ�ָ��
 */
Motor_Measure_t *Get_Pitch_Motor_Feedback_Data(void)
{
    return &gm6020_feedback_data[yaw_motor_index];
}

/**
 * @brief                       ���ؽ����� yaw ��� gm6020 ���������
 * @param                       void
 * @return {Motor_Measure_t*}   ָ�� yaw �����ṹ�����ݵ�ָ��
 */
Motor_Measure_t *Get_Yaw_Motor_Feedback_Data(void)
{
    return &gm6020_feedback_data[pitch_motor_index];
}
