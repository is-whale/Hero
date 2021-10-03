#include "can2_device.h"

static Pid_Position_t motor_yaw_speed_pid = NEW_POSITION_PID(12, 0, 2, 2000, 16000, 0, 1000, 500); //yaw����ٶ�PID
// static Pid_Position_t motor_yaw_angle_pid = NEW_POSITION_PID(2.4, 0.01, 1.8, 5, 125, 0, 3000, 500); //yaw����Ƕ�PID
static Pid_Position_t motor_pitch_speed_pid = NEW_POSITION_PID(12, 0, 2, 2000, 16000, 0, 1000, 500);		//pitch����ٶ�PID
static Pid_Position_t motor_pitch_angle_pid = NEW_POSITION_PID(0.25, 0.018, 0.005, 100, 300, 0, 3000, 500); //pitch����Ƕ�PID
static Pid_Position_t wave_motor_left_speed_pid = NEW_POSITION_PID(11, 0, 5, 2000, 16000, 0, 1000, 500);
static Pid_Position_t wave_motor_right_speed_pid = NEW_POSITION_PID(11, 0, 5, 2000, 16000, 0, 1000, 500);

static uint8_t can2_rxd_data_buffer[8];	   ///< �������������ܵ��������ԭʼ����
static CAN_RxHeaderTypeDef can2_rx_header; ///< �������������� HAL �⺯���������ݣ���Ź��� CAN �������ݵ� ID �ŵ���Ϣ

/**
 * @brief 			can2 ���ջص�����,�ڸú����ڣ�ֻ�������ݣ�����������ݽ����ڸ�����Ҫ����������
 * @param[in]		void
 * @retval			void
 */
void Can2_Rx_FIFO0_IT_Callback(void)
{
	HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &can2_rx_header, can2_rxd_data_buffer);
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

/**
 * @brief 				���� can2 ���յ��������׵�ַ���������������
 * @param[in]			void
 * @return uint8_t* 	ָ��������ݵ������׵�ַָ��
 */
uint8_t *Get_CAN2_Rxd_Buffer(void)
{
	return can2_rxd_data_buffer;
}

/**
 * @brief 									������̨������ٶȣ�����һ����̨��ͨ���ǶȽ��п��ƣ�����������ٶ�ֵ��ʵ�Ǿ����Ƕ� pid ����õ����ٶ�ֵ
 * @param yaw_speed 						���õ� yaw �������ٶ�ֵ
 * @param pitch_speed 						���õ� pitch �������ٶ�ֵ
 * @param yaw_motor_parsed_feedback_data 	ָ�� yaw ����������Ľṹ��ָ��
 * @param pitch_motor_parsed_feedback_data  ָ�� pitch ����������Ľṹ��ָ��
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
 * @brief 				���� pitch �����ĽǶ�
 * @param angle 		��ǰ��Ҫ�趨�ĽǶ�
 * @param down_angle 	��ͽǶ�
 * @param up_angle 		��߽Ƕ�
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
 * @brief 									������̨����ĽǶ� pid
 * @param tar_angle 						�趨�Ƕ�ֵ
 * @param pitch_motor_parsed_feedback_data  ָ�� pitch ���������ṹ��ָ��
 * @return float 							pid ������
 */
float Calc_Pitch_Angle8191_Pid(float tar_angle, Motor_Measure_t *pitch_motor_parsed_feedback_data)
{
	float pitch_tar_angle = tar_angle;
	float pitch_cur_angle = pitch_motor_parsed_feedback_data->mechanical_angle;
	Handle_Angle8191_PID_Over_Zero(&pitch_tar_angle, &pitch_cur_angle);
	return Pid_Position_Calc(&motor_pitch_angle_pid, pitch_tar_angle, pitch_cur_angle); ///< ���ǵ�һ�� PID�������趨�Ƕ���ʵ�ʽǶ�֮������õ���һ��Ҫ�趨���ٶ�ֵ������Ѿ��ﵽĿ��ֵ�������Ϊ 0
}

/**
 * @brief 								���ò��ֵ�����ٶ�
 * @param speed_left 					��߲��ֵ�����ٶ�
 * @param speed_right 					�ұ߲��ֵ�����ٶ�
 * @param wave_motor_feedback_data 		�������ֵ��������ķ�������
 */
void Set_Wave_Motor_Speed(float speed_left, float speed_right, Motor_Measure_t *wave_motor_feedback_data)
{
	int16_t left = Pid_Position_Calc(&wave_motor_left_speed_pid, speed_left, wave_motor_feedback_data[0].speed_rpm);
	int16_t right = Pid_Position_Calc(&wave_motor_right_speed_pid, speed_right, wave_motor_feedback_data[1].speed_rpm);
	Can2_Send_4Msg(
		CAN_SHOOTER_ALL_ID,
		left,
		right,
		0,
		0);
}
