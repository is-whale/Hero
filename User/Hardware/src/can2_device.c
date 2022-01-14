#include "can2_device.h"

static Pid_Position_t motor_yaw_speed_pid = NEW_POSITION_PID(12, 0, 2, 2000, 16000, 0, 1000, 500); ///< yaw电机速度PID
// static Pid_Position_t motor_yaw_angle_pid = NEW_POSITION_PID(2.4, 0.01, 1.8, 5, 125, 0, 3000, 500); ///< yaw电机角度PID
static Pid_Position_t motor_pitch_speed_pid = NEW_POSITION_PID(20, 0, 3, 2000, 40000, 0, 1000, 500);		///< pitch电机速度PID
static Pid_Position_t motor_pitch_angle_pid = NEW_POSITION_PID(0.25, 0.018, 0.005, 100, 1000, 0, 3000, 500); ///< pitch电机角度PID

static Pid_Position_t friction_motor_left_speed_pid = NEW_POSITION_PID(7, 0, 0.7, 2000, 16383, 0, 1000, 500);
static Pid_Position_t friction_motor_right_speed_pid = NEW_POSITION_PID(7, 0, 0.7, 2000, 16383, 0, 1000, 500);
static Pid_Position_t wave_motor_speed_pid = NEW_POSITION_PID(9, 0, 3, 2000, 16000, 0, 1000, 500);

static Pid_Position_t wave_motor_angle_pid = NEW_POSITION_PID(0.25, 0.018, 0.005, 100, 4500, 0, 3000, 500); ///<  拨轮电机角度PID

static int error_integral = 0;
static uint16_t last_machine_angle = 0;
static uint16_t this_machine_angle = 0;

static uint8_t can2_rxd_data_buffer[8];	   ///< 辅助变量，接受电机反馈的原始数据
static CAN_RxHeaderTypeDef can2_rx_header; ///< 辅助变量，用于 HAL 库函数接受数据，存放关于 CAN 反馈数据的 ID 号等信息

static Motor_Measure_t wave_motor_feedback_data;

/**
 * @brief 			解析波轮电机数据
 * @param[in]		void
 * @retval			void
 */
static void Parse_Wave_Motor_Feedback_Data(void)
{
	static uint8_t flag = 1;
	float last_, this_;
	Calculate_Motor_Data(&wave_motor_feedback_data, can2_rxd_data_buffer);
	if (flag)
	{
		flag = 0;
		last_machine_angle = wave_motor_feedback_data.mechanical_angle;
	}
	this_machine_angle = wave_motor_feedback_data.mechanical_angle;
	this_ = this_machine_angle;
	last_ = last_machine_angle;
	Handle_Angle8191_PID_Over_Zero(&this_, &last_);
	error_integral += (this_ - last_);
	last_machine_angle = this_machine_angle;
}

int *Get_Error_Integral(void)
{
	return &error_integral;
}

uint16_t *Get_Last_Machine_Angle(void)
{
	return &last_machine_angle;
}

uint16_t *Get_This_Machine_Angle(void)
{
	return &this_machine_angle;
}

/**
 * @brief 			can2 接收回调函数,在该函数内，只接受数据，而具体的数据解析在各自需要的任务里面
 * @param[in]		void
 * @retval			void
 */
void Can2_Rx_FIFO0_IT_Callback(void)
{
	HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &can2_rx_header, can2_rxd_data_buffer);
	///< 所有 can 数据，只有波轮电机的数据是直接在中断中进行解析的，因为要实时获取电机的绝对角度
	if (can2_rx_header.StdId == CAN_3508_WAVE_ID)
	{
		Parse_Wave_Motor_Feedback_Data();
	}
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
 * @brief 		返回中断解析后的波轮电机结构体指针
 * @param[in]	void
 * @return 		Motor_Measure_t* 解析后的波轮电机结构体指针
 */
Motor_Measure_t *Get_Wave_Motor_Paresed_Data(void)
{
	return &wave_motor_feedback_data;
}

/**
 * @brief 									设置云台电机的速度，但是一般云台是通过角度进行控制，这里输入的速度值
 * 											其实是经过角度 pid 计算得到的速度值
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
		Pid_Position_Calc(&motor_pitch_speed_pid, pitch_speed, pitch_motor_parsed_feedback_data->speed_rpm),
		0,
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
 * @param pitch_motor_parsed_feedback_data  指向 pitch 轴电机反馈结构体指针（实际值）
 * @return float 							pid 计算结果
 */
float Calc_Pitch_Angle8191_Pid(float tar_angle, Motor_Measure_t *pitch_motor_parsed_feedback_data)
{
	float pitch_tar_angle = tar_angle;
	float pitch_cur_angle = pitch_motor_parsed_feedback_data->mechanical_angle;
	Handle_Angle8191_PID_Over_Zero(&pitch_tar_angle, &pitch_cur_angle);
	return Pid_Position_Calc(&motor_pitch_angle_pid, pitch_tar_angle, pitch_cur_angle); ///< 这是第一层 PID，计算设定角度与实际角度之间的误差，得到下一步要设定的速度值，如果已经达到目标值，则输出为 0
}

float Calc_Wave_Motor_Angle8191_Pid(float tar_angle, float current_angle)
{
	float wave_motor_tar_angle = tar_angle;
	float wave_motor_cur_angle = current_angle;
	//Handle_Angle8191_PID_Over_Zero(&tar_angle, &current_angle);
	return Pid_Position_Calc(&wave_motor_angle_pid, wave_motor_tar_angle, wave_motor_cur_angle); ///< 这是第一层 PID，计算设定角度与实际角度之间的误差，得到下一步要设定的速度值，如果已经达到目标值，则输出为 0
}


/**
 * @brief 								设置波轮电机的速度
 * @param speed_left 					左边波轮电机的速度
 * @param speed_right 					右边波轮电机的速度
 * @param wave_motor_feedback_data 		两个波轮电机解析后的反馈数据
 */
void Set_Friction_Motor_Speed(float speed_left, float speed_right, Motor_Measure_t *friction_motor_feedback_data)
{
	int16_t left = Pid_Position_Calc(&friction_motor_left_speed_pid, speed_left, friction_motor_feedback_data[0].speed_rpm);
	int16_t right = Pid_Position_Calc(&friction_motor_right_speed_pid, speed_right, friction_motor_feedback_data[1].speed_rpm);
	Can2_Send_4Msg(
		CAN_SHOOTER_ALL_ID,
		left,
		right,
		0,
		0);
}

void Set_Wave_Motor_Speed(float wave_motor_speed, Motor_Measure_t *wave_motor_feedback_data)
{
	int16_t wave_motor_pid_out_speed = Pid_Position_Calc(&wave_motor_speed_pid, wave_motor_speed, wave_motor_feedback_data->speed_rpm);
	Can2_Send_4Msg(
		0x1FF,
		0,
		wave_motor_pid_out_speed,
		0,
		0);
}
