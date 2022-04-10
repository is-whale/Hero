#include "can2_device.h"
#include "gimbal_task.h"
#include "monitor_task.h"

// extern float easy_pid_p, easy_pid_i, easy_pid_d;
// static Pid_Position_t motor_yaw_speed_pid = NEW_POSITION_PID(1800, 0.8, 0.2, 5000, 30000, 0, 1000, 500); 		///< yaw����ٶ�PID
// static Pid_Position_t motor_yaw_speed_pid = NEW_POSITION_PID(600, 0, 0.02, 5000, 28000, 0, 1000, 500); ///< yaw����ٶ�PID(���������)
// static Pid_Position_t motor_yaw_angle_pid = NEW_POSITION_PID(0.05, 0, 0, 5, 125, 0, 3000, 500);		///< yaw����Ƕ�PID
/* ��ʱ������ */
static Pid_Position_t motor_yaw_speed_pid = NEW_POSITION_PID(2000, 0.6, 0.2, 5000, 29000, 0, 1000, 500); //yaw����ٶ�PID
static Pid_Position_t motor_yaw_angle_pid = NEW_POSITION_PID(0.2, 0.0, 0.0, 5, 125, 0, 3000, 500);		 //yaw����Ƕ�PID

static Pid_Position_t motor_pitch_speed_pid = NEW_POSITION_PID(350, 25, 0, 220, 15000, 0, 1000, 500); ///< pitch����ٶ�PID
// static Pid_Position_t motor_pitch_speed_pid = NEW_POSITION_PID(300, 27, 0, 220, 30000, 0, 1000, 500);		 	///< pitch����ٶ�PID
static Pid_Position_t motor_pitch_angle_pid = NEW_POSITION_PID(0.03, 0.02, 0.00, 100, 50, 0, 3000, 500); ///< pitch����Ƕ�PID
// static Pid_Position_t motor_pitch_angle_pid = NEW_POSITION_PID(0.4, 0.008, 0.005, 100, 1000, 0, 1000, 500); ///< pitch����Ƕ�PID

static Pid_Position_t friction_motor_left_speed_pid = NEW_POSITION_PID(7, 0, 0.7, 2000, 16383, 0, 1000, 500);
static Pid_Position_t friction_motor_right_speed_pid = NEW_POSITION_PID(7, 0, 0.7, 2000, 16383, 0, 1000, 500);

static Pid_Position_t wave_motor_speed_pid = NEW_POSITION_PID(9, 0, 3, 2000, 16000, 0, 1000, 500);
static Pid_Position_t wave_motor_angle_pid = NEW_POSITION_PID(0.25, 0.018, 0.005, 100, 4500, 0, 3000, 500); ///<  ���ֵ���Ƕ�PID
/* ������̨pitch�Ƕ�PID���� */
static Pid_Position_t motor_pitch_angle_pid_imu = NEW_POSITION_PID(0.2, 0.0, 0.0, 100, 1000, 0, 3000, 500);
// static Pid_Position_t motor_pitch_speed_pid_imu = NEW_POSITION_PID(100, 0.0, 0.0, 100, 1000, 0, 3000, 500);

static int error_integral = 0;
static uint16_t last_machine_angle = 0;
static uint16_t this_machine_angle = 0;

static uint8_t can2_rxd_data_buffer[8];	   ///< �������������ܵ��������ԭʼ����
static CAN_RxHeaderTypeDef can2_rx_header; ///< �������������� HAL �⺯���������ݣ���Ź��� CAN �������ݵ� ID �ŵ���Ϣ

static Motor_Measure_t wave_motor_feedback_data; ///<���ֵ����������

extern float easy_pid_p, easy_pid_i, easy_pid_d;

// void up_date_pid(void)
// {
// 	motor_yaw_speed_pid.kp = easy_pid_p;
// 	motor_yaw_speed_pid.ki = easy_pid_i;
// 	motor_yaw_speed_pid.kd = easy_pid_d;
// }
/**
 * @brief 			�������ֵ������
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
 * @brief 			can2 ���ջص�����,�ڸú����ڣ�ֻ�������ݣ�����������ݽ����ڸ�����Ҫ����������
 * @param[in]		void
 * @retval			void
 */
void Can2_Rx_FIFO0_IT_Callback(void)
{
	HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &can2_rx_header, can2_rxd_data_buffer);
	///< ���� can ���ݣ�ֻ�в��ֵ����������ֱ�����ж��н��н����ģ���ΪҪʵʱ��ȡ����ľ��ԽǶ�
	switch (can2_rx_header.StdId)
	{
	case CAN_3508_WAVE_ID:
	{
		Parse_Wave_Motor_Feedback_Data();
	}
	case CAN_YAW_MOTOR_ID:
	case CAN_PITCH_MOTOR_ID:
	{
		uint8_t i = 0;
		//������ID��
		i = can2_rx_header.StdId - CAN_YAW_MOTOR_ID;
		//֪ͨ����
		Gimbal_Reload(i);
		Can2_Parse_For_Callback();
	}
	}
	// if (can2_rx_header.StdId == CAN_3508_WAVE_ID)
	// {
	// 	Parse_Wave_Motor_Feedback_Data();
	// }
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
 * @brief 		�����жϽ�����Ĳ��ֵ���ṹ��ָ��
 * @param[in]	void
 * @return 		Motor_Measure_t* ������Ĳ��ֵ���ṹ��ָ��
 */
Motor_Measure_t *Get_Wave_Motor_Paresed_Data(void)
{
	return &wave_motor_feedback_data;
}

/**
 * @brief 									��̨����PID���ٶȻ������Լ���CAN2���ͽ��������̨����ٶ�ֵ
 * @param yaw_speed 						���õ� yaw �������ٶ�ֵ
 * @param pitch_speed 						���õ� pitch �������ٶ�ֵ
 * @param yaw_motor_parsed_feedback_data 	ָ�� yaw ����������Ľṹ��ָ��
 * @param pitch_motor_parsed_feedback_data  ָ�� pitch ����������Ľṹ��ָ��
 */
//void Set_Gimbal_Motors_Speed(float yaw_speed, float pitch_speed, Motor_Measure_t *yaw_motor_parsed_feedback_data, Motor_Measure_t *pitch_motor_parsed_feedback_data)
//{
//	Can2_Send_4Msg(
//		CAN_GIMBAL_ALL_ID,
//		Pid_Position_Calc(&motor_yaw_speed_pid, yaw_speed, yaw_motor_parsed_feedback_data->speed_rpm),
//		// Pid_Position_Calc(&motor_pitch_speed_pid, pitch_speed, pitch_motor_parsed_feedback_data->speed_rpm),
//		0,
//		0,
//		0);
//		//float new_date = yaw_motor_parsed_feedback_data->speed_rpm);
//	// __printf("%0.2f,%0.2f,%.2f\r\n", yaw_speed, Pid_Position_Calc(&motor_yaw_speed_pid, yaw_speed, yaw_motor_parsed_feedback_data->speed_rpm), yaw_motor_parsed_feedback_data->speed_rpm);
//}
/**
 * @brief 									��̨����PID���ٶȻ������Լ���CAN2���ͽ��������̨����ٶ�ֵ
 * @param yaw_speed 						���õ� yaw �������ٶ�ֵ
 * @param pitch_speed 						���õ� pitch �������ٶ�ֵ
 * @param yaw_motor_parsed_feedback_data 	ָ�� yaw ����������Ľṹ��ָ��
 * @param pitch_motor_parsed_feedback_data  ָ�� pitch ����������Ľṹ��ָ��
 */
void Set_Gimbal_Motors_Speed(float yaw_speed, float pitch_speed, float yaw_speed_rpm, float pitch_speed_rpm)
{
	Can2_Send_4Msg(
		CAN_GIMBAL_ALL_ID,
		Pid_Position_Calc(&motor_yaw_speed_pid, yaw_speed, yaw_speed_rpm),
		Pid_Position_Calc(&motor_pitch_speed_pid, pitch_speed, pitch_speed_rpm),
		0,
		0);
		/* ����ʹ�� */
	//float new_date = yaw_motor_parsed_feedback_data->speed_rpm);
	// __printf("%0.2f,%0.2f,%.2f\r\n", yaw_speed, Pid_Position_Calc(&motor_yaw_speed_pid, yaw_speed, yaw_speed_rpm), yaw_speed_rpm);
}

/**
 * @brief 				���� pitch �����ĽǶ�
 * @param angle 		��ǰ�Ƕ�Ŀ��ֵ
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
 * @brief 									������̨Yaw����ĽǶȻ����
 * @param tar_angle 						�趨�Ƕ�ֵ(���ʺ��ͨ��ֵ)
 * @param tar_angle							ʵ�ʽǶ�ֵ�����䷴���Ƕ�ֵ���ռ�����ϵ�µľ��ԽǶȣ�
 * @param pitch_motor_parsed_feedback_data  ָ�� yaw ���������ṹ��ָ�루ʵ��ֵ��
 * @return float 							����pid�ǶȻ����
 */
float Calc_Yaw_Angle360_Pid(float tar_angle, float cur_angle)
{
	float yaw_tar_angle = tar_angle; ///< ch0 ͨ��ֵ
	float yaw_cur_angle = cur_angle; ///<�����Ƿ���ֵ

	Handle_Angle360_PID_Over_Zero(&yaw_tar_angle, &yaw_cur_angle);
	return Pid_Position_Calc(&motor_yaw_angle_pid, yaw_tar_angle, yaw_cur_angle);
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

float Calc_Wave_Motor_Angle8191_Pid(float tar_angle, float current_angle)
{
	float wave_motor_tar_angle = tar_angle;
	float wave_motor_cur_angle = current_angle;
	// Handle_Angle8191_PID_Over_Zero(&tar_angle, &current_angle);
	return Pid_Position_Calc(&wave_motor_angle_pid, wave_motor_tar_angle, wave_motor_cur_angle); ///< ���ǵ�һ�� PID�������趨�Ƕ���ʵ�ʽǶ�֮������õ���һ��Ҫ�趨���ٶ�ֵ������Ѿ��ﵽĿ��ֵ�������Ϊ 0
}

/**
 * @brief 								���ò��ֵ�����ٶ�(�����ٶȻ�����)
 * @param speed_left 					��߲��ֵ�����ٶ�
 * @param speed_right 					�ұ߲��ֵ�����ٶ�
 * @param wave_motor_feedback_data 		�������ֵ��������ķ�������
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

/**
 * @brief 								����������̨ģʽ�ĽǶȻ�PID
 * @param tar_angle 					Ŀ��Ƕ�
 * @param imu_on_broad 					�����Ƕȣ�������pit��
 * @retval 								�ǶȻ����
 */
float Calc_Pitch_Angle8191_Imu_Pid(float tar_angle, Imu_t *imu_on_broad)
{
	float pitch_tar_angle = tar_angle;
	float pitch_cur_angle = imu_on_broad->pit * 500.0f;
	Handle_Angle8191_PID_Over_Zero(&pitch_tar_angle, &pitch_cur_angle);
	return Pid_Position_Calc(&motor_pitch_angle_pid_imu, pitch_tar_angle, pitch_cur_angle); ///< ���ǵ�һ�� PID�������趨�Ƕ���ʵ�ʽǶ�֮������õ���һ��Ҫ�趨���ٶ�ֵ������Ѿ��ﵽĿ��ֵ�������Ϊ 0
}
