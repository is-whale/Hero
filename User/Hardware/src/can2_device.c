#include "can2_device.h"

static Motor_Measure_t gm6020_feedback_data[2]; ///< �洢�������������̨��������ݣ�һ��������
static uint8_t can2_rxd_data_buffer[8];			///< �������������ܵ��������ԭʼ����
static const uint8_t yaw_motor_index = 0;		///< yaw ����̨����������е��±�
static const uint8_t pitch_motor_index = 1;		///< pitch ����̨�ڵ���е��±�
static CAN_RxHeaderTypeDef can2_rx_header;		///< �������������� HAL �⺯���������ݣ���Ź��� CAN �������ݵ� ID �ŵ���Ϣ

void Can2_Rx_FIFO0_IT_Callback(void)
{
	HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &can2_rx_header, can2_rxd_data_buffer);
	switch (can2_rx_header.StdId)
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
	case 0:
	{
		// TODO ���� CAN2 �����ϵ����ݷ�������
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

/**
 * @brief                       ���ؽ����� yaw ��� gm6020 ���������
 * @param                       void
 * @return {Motor_Measure_t*}   ָ�� yaw �����ṹ�����ݵ�ָ��
 */
Motor_Measure_t *Get_Gimbal_Motor_Feedback_Data(void)
{
	return gm6020_feedback_data;
}

/**
 * @brief                   ���� pitch ����̨����ڴ洢��������е��±�
 * @param[in]               void
 * @return const uint8_t*   ָ���±������
 */
const uint8_t *Get_Pitch_Motor_Index(void)
{
	return &pitch_motor_index;
}

/**
 * @brief                   ���� yaw ����̨����ڴ洢��������е��±�
 * @param[in]               void
 * @return const uint8_t*   ָ���±������
 */
const uint8_t *Get_Yaw_Motor_Index(void)
{
	return &yaw_motor_index;
}

/**
 * @brief                           ���� CAN_RxHeaderTypeDef ����̨����ڴ洢��������е��±�
 * @param[in]                       void
 * @return CAN_RxHeaderTypeDef*     ָ���±������
 */
CAN_RxHeaderTypeDef *Get_CAN2_Rx_Header(void)
{
	return &can2_rx_header;
}

uint8_t *Get_CAN2_Rxd_Buffer(void)
{
	return can2_rxd_data_buffer;
}
