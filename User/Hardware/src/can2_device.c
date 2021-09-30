#include "can2_device.h"

static Pid_Position_t motor_yaw_speed_pid = NEW_POSITION_PID(12, 0, 2, 2000, 16000, 0, 1000, 500);	//yaw电机速度PID
// static Pid_Position_t motor_yaw_angle_pid = NEW_POSITION_PID(2.4, 0.01, 1.8, 5, 125, 0, 3000, 500); //yaw电机角度PID

static Pid_Position_t motor_pitch_speed_pid = NEW_POSITION_PID(12, 0, 2, 2000, 16000, 0, 1000, 500);		//pitch电机速度PID
// static Pid_Position_t motor_pitch_angle_pid = NEW_POSITION_PID(0.25, 0.018, 0.005, 100, 300, 0, 3000, 500); //pitch电机角度PID

static Motor_Measure_t gm6020_feedback_data[2]; ///< 存储解析后的两个云台电机的数据，一般是两个
static uint8_t can2_rxd_data_buffer[8];			///< 辅助变量，接受电机反馈的原始数据
static const uint8_t yaw_motor_index = 0;		///< yaw 轴云台电机在数组中的下标
static const uint8_t pitch_motor_index = 1;		///< pitch 轴云台在电机中的下标
static CAN_RxHeaderTypeDef can2_rx_header;		///< 辅助变量，用于 HAL 库函数接受数据，存放关于 CAN 反馈数据的 ID 号等信息

void Can2_Rx_FIFO0_IT_Callback(void)
{
	HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &can2_rx_header, can2_rxd_data_buffer);
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

/**
 * @brief                       返回解析后 yaw 轴的 gm6020 电机的数据
 * @param                       void
 * @return {Motor_Measure_t*}   指向 yaw 轴电机结构题数据的指针
 */
Motor_Measure_t *Get_Gimbal_Motor_Feedback_Data(void)
{
	return gm6020_feedback_data;
}

/**
 * @brief                   返回 pitch 轴云台电机在存储电机数组中的下标
 * @param[in]               void
 * @return const uint8_t*   指向下标的索引
 */
const uint8_t *Get_Pitch_Motor_Index(void)
{
	return &pitch_motor_index;
}

/**
 * @brief                   返回 yaw 轴云台电机在存储电机数组中的下标
 * @param[in]               void
 * @return const uint8_t*   指向下标的索引
 */
const uint8_t *Get_Yaw_Motor_Index(void)
{
	return &yaw_motor_index;
}

/**
 * @brief                           返回 CAN_RxHeaderTypeDef 轴云台电机在存储电机数组中的下标
 * @param[in]                       void
 * @return CAN_RxHeaderTypeDef*     指向下标的索引
 */
CAN_RxHeaderTypeDef *Get_CAN2_Rx_Header(void)
{
	return &can2_rx_header;
}

uint8_t *Get_CAN2_Rxd_Buffer(void)
{
	return can2_rxd_data_buffer;
}

void Set_Gimbal_Motors_Speed(float yaw_speed, float pitch_speed, Motor_Measure_t *yaw_motor_parsed_feedback_data, Motor_Measure_t *pitch_motor_parsed_feedback_data)
{
	Can2_Send_4Msg(
		CAN_GIMBAL_ALL_ID,
		Pid_Position_Calc(&motor_yaw_speed_pid, yaw_speed, yaw_motor_parsed_feedback_data->speed_rpm),
		Pid_Position_Calc(&motor_pitch_speed_pid, pitch_speed, pitch_motor_parsed_feedback_data->speed_rpm),
		0,
		0);
}
