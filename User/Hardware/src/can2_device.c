#include "can2_device.h"

<<<<<<< HEAD
static Pid_Position_t motor_yaw_speed_pid = NEW_POSITION_PID(12, 0, 2, 2000, 16000, 0, 1000, 500); ///< yawç”µæœºé€Ÿåº¦PID
// static Pid_Position_t motor_yaw_angle_pid = NEW_POSITION_PID(2.4, 0.01, 1.8, 5, 125, 0, 3000, 500); ///< yawç”µæœºè§’åº¦PID

//static Pid_Position_t motor_pitch_speed_pid = NEW_POSITION_PID(20, 0, 3, 2000, 50000, 0, 1000, 500);		///< pitchç”µæœºé€Ÿåº¦PID  åŽŸæ¥çš„
static Pid_Position_t motor_pitch_speed_pid = NEW_POSITION_PID( 10, 0, 3,220, 30000, 0, 1000, 500);		///< pitchç”µæœºé€Ÿåº¦PID

//static Pid_Position_t motor_pitch_angle_pid = NEW_POSITION_PID(0.25, 0.018, 0.005, 100, 1000, 0, 3000, 500); ///< pitchç”µæœºè§’åº¦PID
static Pid_Position_t motor_pitch_angle_pid = NEW_POSITION_PID(0.25,0.018,0.005,100, 300, 0, 3000, 500); ///< pitchç”µæœºè§’åº¦PID
=======
static Pid_Position_t motor_yaw_speed_pid = NEW_POSITION_PID(1800, 0.8, 0.2, 5000, 30000, 0, 1000, 500); ///< yawµç»úËÙ¶ÈPID
static Pid_Position_t motor_yaw_angle_pid = NEW_POSITION_PID(2.4, 0.01, 1.8, 5, 125, 0, 3000, 500); ///< yawµç»ú½Ç¶ÈPID
static Pid_Position_t motor_pitch_speed_pid = NEW_POSITION_PID(20, 0, 3, 2000, 40000, 0, 1000, 500);		///< pitchµç»úËÙ¶ÈPID
static Pid_Position_t motor_pitch_angle_pid = NEW_POSITION_PID(0.25, 0.018, 0.005, 100, 1000, 0, 3000, 500); ///< pitchµç»ú½Ç¶ÈPID
>>>>>>> 29a6636b20a8181e2d633db14557fb4204aafeaf

static Pid_Position_t friction_motor_left_speed_pid = NEW_POSITION_PID(7, 0, 0.7, 2000, 16383, 0, 1000, 500);
static Pid_Position_t friction_motor_right_speed_pid = NEW_POSITION_PID(7, 0, 0.7, 2000, 16383, 0, 1000, 500);
static Pid_Position_t wave_motor_speed_pid = NEW_POSITION_PID(9, 0, 3, 2000, 16000, 0, 1000, 500);

static Pid_Position_t wave_motor_angle_pid = NEW_POSITION_PID(0.25, 0.018, 0.005, 100, 4500, 0, 3000, 500); ///<  ²¦ÂÖµç»ú½Ç¶ÈPID

static int error_integral = 0;
static uint16_t last_machine_angle = 0;
static uint16_t this_machine_angle = 0;

static uint8_t can2_rxd_data_buffer[8];	   ///< ¸¨Öú±äÁ¿£¬½ÓÊÜµç»ú·´À¡µÄÔ­Ê¼Êý¾Ý
static CAN_RxHeaderTypeDef can2_rx_header; ///< ¸¨Öú±äÁ¿£¬ÓÃÓÚ HAL ¿âº¯Êý½ÓÊÜÊý¾Ý£¬´æ·Å¹ØÓÚ CAN ·´À¡Êý¾ÝµÄ ID ºÅµÈÐÅÏ¢

static Motor_Measure_t wave_motor_feedback_data;

/**
 * @brief 			½âÎö²¨ÂÖµç»úÊý¾Ý
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
 * @brief 			can2 ½ÓÊÕ»Øµ÷º¯Êý,ÔÚ¸Ãº¯ÊýÄÚ£¬Ö»½ÓÊÜÊý¾Ý£¬¶ø¾ßÌåµÄÊý¾Ý½âÎöÔÚ¸÷×ÔÐèÒªµÄÈÎÎñÀïÃæ
 * @param[in]		void
 * @retval			void
 */
void Can2_Rx_FIFO0_IT_Callback(void)
{
	HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &can2_rx_header, can2_rxd_data_buffer);
	///< ËùÓÐ can Êý¾Ý£¬Ö»ÓÐ²¨ÂÖµç»úµÄÊý¾ÝÊÇÖ±½ÓÔÚÖÐ¶ÏÖÐ½øÐÐ½âÎöµÄ£¬ÒòÎªÒªÊµÊ±»ñÈ¡µç»úµÄ¾ø¶Ô½Ç¶È
	if (can2_rx_header.StdId == CAN_3508_WAVE_ID)
	{
		Parse_Wave_Motor_Feedback_Data();
	}
}

/**
 * @brief                           ·µ»Ø CAN_RxHeaderTypeDef ÖáÔÆÌ¨µç»úÔÚ´æ´¢µç»úÊý×éÖÐµÄÏÂ±ê
 * @param[in]                       void
 * @return CAN_RxHeaderTypeDef*     Ö¸ÏòÏÂ±êµÄË÷Òý
 */
CAN_RxHeaderTypeDef *Get_CAN2_Rx_Header(void)
{
	return &can2_rx_header;
}

/**
 * @brief 				·µ»Ø can2 ½ÓÊÕµ½µÄÊý×éÊ×µØÖ·£¬¹©¸÷¸öÈÎÎñ½âÎö
 * @param[in]			void
 * @return uint8_t* 	Ö¸Ïò½ÓÊÕÊý¾ÝµÄÊý×éÊ×µØÖ·Ö¸Õë
 */
uint8_t *Get_CAN2_Rxd_Buffer(void)
{
	return can2_rxd_data_buffer;
}

/**
 * @brief 		·µ»ØÖÐ¶Ï½âÎöºóµÄ²¨ÂÖµç»ú½á¹¹ÌåÖ¸Õë
 * @param[in]	void
 * @return 		Motor_Measure_t* ½âÎöºóµÄ²¨ÂÖµç»ú½á¹¹ÌåÖ¸Õë
 */
Motor_Measure_t *Get_Wave_Motor_Paresed_Data(void)
{
	return &wave_motor_feedback_data;
}

/**
 * @brief 									ÉèÖÃÔÆÌ¨µç»úµÄËÙ¶È£¬µ«ÊÇÒ»°ãÔÆÌ¨ÊÇÍ¨¹ý½Ç¶È½øÐÐ¿ØÖÆ£¬ÕâÀïÊäÈëµÄËÙ¶ÈÖµ
 * 											ÆäÊµÊÇ¾­¹ý½Ç¶È pid ¼ÆËãµÃµ½µÄËÙ¶ÈÖµ
 * @param yaw_speed 						ÉèÖÃµÄ yaw Öáµç»úµÄËÙ¶ÈÖµ
 * @param pitch_speed 						ÉèÖÃµÄ pitch Öáµç»úµÄËÙ¶ÈÖµ
 * @param yaw_motor_parsed_feedback_data 	Ö¸Ïò yaw Öáµç»ú½âÎöºóµÄ½á¹¹ÌåÖ¸Õë
 * @param pitch_motor_parsed_feedback_data  Ö¸Ïò pitch Öáµç»ú½âÎöºóµÄ½á¹¹ÌåÖ¸Õë
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
 * @brief 				ÏÞÖÆ pitch Öáµç»úµÄ½Ç¶È
 * @param angle 		µ±Ç°½«ÒªÉè¶¨µÄ½Ç¶È
 * @param down_angle 	×îµÍ½Ç¶È
 * @param up_angle 		×î¸ß½Ç¶È
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
 * @brief 									¼ÆËãÔÆÌ¨Yawµç»úµÄ½Ç¶È»·Êä³ö
 * @param tar_angle 						Éè¶¨½Ç¶ÈÖµ(±¶ÂÊºóµÄÍ¨µÀÖµ)
 * @param tar_angle							Êµ¼Ê½Ç¶ÈÖµ£¨ÍÑÂä·´À¡½Ç¶ÈÖµ£¬¿Õ¼ä×ø±êÏµÏÂµÄ¾ø¶Ô½Ç¶È£© 
 * @param pitch_motor_parsed_feedback_data  Ö¸Ïò yaw Öáµç»ú·´À¡½á¹¹ÌåÖ¸Õë£¨Êµ¼ÊÖµ£©
 * @return float 							´®¼¶pid½Ç¶È»·Êä³ö
 */
float Calc_Yaw_Angle360_Pid(float tar_angle, float cur_angle)
{
	float yaw_tar_angle = tar_angle;///<ch0 Í¨µÀÖµ
	float yaw_cur_angle = cur_angle;///<ÍÓÂÝÒÇ·´À¡Öµ
	
	Handle_Angle360_PID_Over_Zero(&yaw_tar_angle, &yaw_cur_angle);
	return Pid_Position_Calc(&motor_yaw_angle_pid, yaw_tar_angle, yaw_cur_angle);
}

/**
 * @brief 									¼ÆËãÔÆÌ¨µç»úµÄ½Ç¶È pid
 * @param tar_angle 						Éè¶¨½Ç¶ÈÖµ
 * @param pitch_motor_parsed_feedback_data  Ö¸Ïò pitch Öáµç»ú·´À¡½á¹¹ÌåÖ¸Õë£¨Êµ¼ÊÖµ£©
 * @return float 							pid ¼ÆËã½á¹û
 */
float Calc_Pitch_Angle8191_Pid(float tar_angle, Motor_Measure_t *pitch_motor_parsed_feedback_data)
{
	float pitch_tar_angle = tar_angle;
	float pitch_cur_angle = pitch_motor_parsed_feedback_data->mechanical_angle;
	Handle_Angle8191_PID_Over_Zero(&pitch_tar_angle, &pitch_cur_angle);
	return Pid_Position_Calc(&motor_pitch_angle_pid, pitch_tar_angle, pitch_cur_angle); ///< ÕâÊÇµÚÒ»²ã PID£¬¼ÆËãÉè¶¨½Ç¶ÈÓëÊµ¼Ê½Ç¶ÈÖ®¼äµÄÎó²î£¬µÃµ½ÏÂÒ»²½ÒªÉè¶¨µÄËÙ¶ÈÖµ£¬Èç¹ûÒÑ¾­´ïµ½Ä¿±êÖµ£¬ÔòÊä³öÎª 0
}

float Calc_Wave_Motor_Angle8191_Pid(float tar_angle, float current_angle)
{
	float wave_motor_tar_angle = tar_angle;
	float wave_motor_cur_angle = current_angle;
	//Handle_Angle8191_PID_Over_Zero(&tar_angle, &current_angle);
	return Pid_Position_Calc(&wave_motor_angle_pid, wave_motor_tar_angle, wave_motor_cur_angle); ///< ÕâÊÇµÚÒ»²ã PID£¬¼ÆËãÉè¶¨½Ç¶ÈÓëÊµ¼Ê½Ç¶ÈÖ®¼äµÄÎó²î£¬µÃµ½ÏÂÒ»²½ÒªÉè¶¨µÄËÙ¶ÈÖµ£¬Èç¹ûÒÑ¾­´ïµ½Ä¿±êÖµ£¬ÔòÊä³öÎª 0
}


/**
 * @brief 								ÉèÖÃ²¨ÂÖµç»úµÄËÙ¶È
 * @param speed_left 					×ó±ß²¨ÂÖµç»úµÄËÙ¶È
 * @param speed_right 					ÓÒ±ß²¨ÂÖµç»úµÄËÙ¶È
 * @param wave_motor_feedback_data 		Á½¸ö²¨ÂÖµç»ú½âÎöºóµÄ·´À¡Êý¾Ý
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
