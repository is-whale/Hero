#include "can2_device.h"

static Pid_Position_t motor_yaw_speed_pid = NEW_POSITION_PID(12, 0, 2, 2000, 16000, 0, 1000, 500); //yaw电机速度PID
// static Pid_Position_t motor_yaw_angle_pid = NEW_POSITION_PID(2.4, 0.01, 1.8, 5, 125, 0, 3000, 500); //yaw电机角度PID
static Pid_Position_t motor_pitch_speed_pid = NEW_POSITION_PID(12, 0, 2, 2000, 16000, 0, 1000, 500);		//pitch电机速度PID
static Pid_Position_t motor_pitch_angle_pid = NEW_POSITION_PID(0.25, 0.018, 0.005, 100, 300, 0, 3000, 500); //pitch电机角度PID

static uint8_t can2_rxd_data_buffer[8];	   ///< 辅助变量，接受电机反馈的原始数据
static CAN_RxHeaderTypeDef can2_rx_header; ///< 辅助变量，用于 HAL 库函数接受数据，存放关于 CAN 反馈数据的 ID 号等信息

/**
 * @brief 			can2 接收回调函数,在该函数内，只接受数据，而具体的数据解析在各自需要的任务里面
 * @param[in]		void
 * @retval			void
 */
void Can2_Rx_FIFO0_IT_Callback(void)
{
	HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &can2_rx_header, can2_rxd_data_buffer);
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

/**
 * @brief 				返回 can2 接收到的数组首地址，供各个任务解析
 * @param[in]			void
 * @return uint8_t* 	指向接收数据的数组首地址指针
 */
uint8_t *Get_CAN2_Rxd_Buffer(void)
{
	return can2_rxd_data_buffer;
}

/**
 * @brief 									设置云台电机的速度，但是一般云台是通过角度进行控制，这里输入的速度值其实是经过角度 pid 计算得到的速度值
 * @param yaw_speed 						设置的 yaw 轴电机的速度值
 * @param pitch_speed 						设置的 pitch 轴电机的速度值
 * @param yaw_motor_parsed_feedback_data 	指向 yaw 轴电机解析后的结构体指针
 * @param pitch_motor_parsed_feedback_data  指向 pitch 轴电机解析后的结构体指针
 */
void Set_Gimbal_Motors_Speed(float yaw_speed, float pitch_speed, Motor_Measure_t *yaw_motor_parsed_feedback_data, Motor_Measure_t *pitch_motor_parsed_feedback_data)
{
	Can2_Send_4Msg(
		CAN_GIMBAL_ALL_ID,
		Pid_Position_Calc(&motor_yaw_speed_pid, yaw_speed, yaw_motor_parsed_feedback_data->speed_rpm),
		0,
		Pid_Position_Calc(&motor_pitch_speed_pid, pitch_speed, pitch_motor_parsed_feedback_data->speed_rpm),
		0);
}

/**
 * @brief 				限制 pitch 轴电机的角度
 * @param angle 		当前将要设定的角度
 * @param down_angle 	最低角度
 * @param up_angle 		最高角度
 */
void Pitch_Angle_Limit(float *angle, float down_angle, float up_angle)
{
	if (down_angle > up_angle)
	{
		if (*angle > down_angle)
			*angle = down_angle;
		else if (*angle < up_angle)
			*angle = up_angle;
	}
	else if (down_angle <= up_angle)
	{
		if (*angle < down_angle)
			*angle = down_angle;
		else if (*angle > up_angle)
			*angle = up_angle;
	}
}
/**
 * @brief 									计算云台电机的角度 pid
 * @param tar_angle 						设定角度值
 * @param pitch_motor_parsed_feedback_data  指向 pitch 轴电机反馈结构体指针
 * @return float 							pid 计算结果
 */
float Calc_Pitch_Angle8191_Pid(float tar_angle, Motor_Measure_t *pitch_motor_parsed_feedback_data)
{
	float pitch_tar_angle = tar_angle;
	float pitch_cur_angle = pitch_motor_parsed_feedback_data->mechanical_angle;
	Handle_Angle8191_PID_Over_Zero(&pitch_tar_angle, &pitch_cur_angle);
	// debug_print("setangle: %.2f, cur_angle: %.2f ",pitch_tar_angle,pitch_cur_angle);
	///< 这是第一层 PID，计算设定角度与实际角度之间的误差，得到下一步要设定的速度值，如果已经达到目标值，则输出为 0
	return Pid_Position_Calc(&motor_pitch_angle_pid, pitch_tar_angle, pitch_cur_angle);
}
