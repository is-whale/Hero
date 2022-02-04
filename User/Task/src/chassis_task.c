/**
 * @file chassis_task.c
 * @brief ��������
 * @version 0.1
 * @date 2021-9-28
 * @copyright Copyright (c) 2021
 */
#include "chassis_task.h"

#define CHASSIS_SPEED_ZERO  0                                           ///<�رյ����ٶ� 1Ϊ����
#define OUTPUT_LIMIT(data, limit) Float_Constraion(data, -limit, limit) ///<����޷�
#define CHASSIS_MOTOR_DEFAULT_BASE_RATE 5.5f                            //����Ĭ���ٶȵı���
#define CHASSIS_MOTOR_GYRO_BASE_RATE 5.0f 

#define POWER_LIMIT_FORM_SYSTEM 50.0f///<�������Ҫ���ɲ���ϵͳ�����Ĺ������ƣ�Ϊ�˱��⾯����ʱʹ�����ִ���

static Pid_Position_t chassis_follow_pid = NEW_POSITION_PID(0.26, 0, 0.8, 5000, 500, 0, 1000, 500);         ///< ���̸���PID
static float chassis_motor_boost_rate = 1.0f;                                                               ///<������Ӧ�������ģ������ٶȱ��ʣ�
static const float motor_speed_multiple = 13.5;

static Rc_Ctrl_t *rc_data_pt;                               ///< ָ��������ң�����ṹ��ָ��
static Robot_control_data_t *robot_mode_data_pt;            ///< ָ�������Ļ�����ģʽ�ṹ��ָ��(Ҳ�������������ͨ����ֵ)
static Motor_Measure_t *chassis_motor_feedback_parsed_data; ///< ������ĵ��̵������
static Motor_Measure_t *gimbal_motor_feedback_parsed_data;  ///< ���������̨�������
static const uint8_t *yaw_motor_index;                      ///< yaw ��������̨��������е��±�
static const uint8_t *pitch_motor_index;                    ///< pitch ��������̨��������е��±�
/*��������*/
void Chassis_Init(void);                                                ///<���̳�ʼ����������
static uint16_t Calc_Gyro_Speed_By_Power_Limit(uint16_t power_limit);   ///<���㹦�������µ�С���ݻ��ߵ��̸���ʱ�ĵ���ٶ�Ŀ��ֵ
void Calc_Gyro_Motors_Speed(float *motors_speed, float rotate_speed,float move_direction, float x_move_speed, float y_move_speed);

void StartChassisTask(void const *argument)
{
    static float chassis_motor_speed[4] = {0.0, 0.0, 0.0, 0.0};                   ///<������ʱ�洢����ٶ�
    float follow_pid_output;///<����    PID���
    
    Chassis_Init();///<���̳�ʼ��

    osDelay(1000); 

for (;;)
{
        ///<ѡ������豸    ����&ң����
        /*����ģʽ*/
    if (robot_mode_data_pt->mode.control_device == mouse_keyboard_device_ENUM) 
    {
            /**
             * @brief ѡ�������̨ģʽ    1���̸���   2С����   3����ģʽ
             *����ģʽ����ʹ�����������ݣ��Ե�ǰ��̨����ϵ��Ϊ��������ϵ����ǰ����������̨�ӽ�Ϊ׼
             *�����������ǹҵ�ʱ���õ�ģʽ��������������û�������������군
            */
        switch(robot_mode_data_pt->mode.mouse_keyboard_chassis_mode)
			{
					case mk_chassis_follow_mode_ENUM:///<���̸���
                {
                    //���濴��Ҫ��װ�ɺ���

                    follow_pid_output = Calc_Chassis_Follow();
              		chassis_motor_speed[0] = robot_mode_data_pt->virtual_rocker.ch2 + robot_mode_data_pt->virtual_rocker.ch3 + follow_pid_output + rc_data_pt->mouse.x/0.38f;
					chassis_motor_speed[1] = robot_mode_data_pt->virtual_rocker.ch2 - robot_mode_data_pt->virtual_rocker.ch3 + follow_pid_output + rc_data_pt->mouse.x/0.38f;
					chassis_motor_speed[2] = -robot_mode_data_pt->virtual_rocker.ch2 + robot_mode_data_pt->virtual_rocker.ch3 + follow_pid_output + rc_data_pt->mouse.x/0.38f;
					chassis_motor_speed[3] = -robot_mode_data_pt->virtual_rocker.ch2 - robot_mode_data_pt->virtual_rocker.ch3 + follow_pid_output + rc_data_pt->mouse.x/0.38f;
                    ///<��һ������
                    chassis_motor_speed[0] *= (float)(CHASSIS_MOTOR_DEFAULT_BASE_RATE * chassis_motor_boost_rate);
					chassis_motor_speed[1] *= (float)(CHASSIS_MOTOR_DEFAULT_BASE_RATE * chassis_motor_boost_rate);
					chassis_motor_speed[2] *= (float)(CHASSIS_MOTOR_DEFAULT_BASE_RATE * chassis_motor_boost_rate);
				    chassis_motor_speed[3] *= (float)(CHASSIS_MOTOR_DEFAULT_BASE_RATE * chassis_motor_boost_rate);
                }

					case mk_chassis_gyro_mode_ENUM:///<����С����

                {
                    ///<С����ʱ�ĵ���ٶȼ���
                    	Calc_Gyro_Motors_Speed(chassis_motor_speed, \
					Calc_Gyro_Speed_By_Power_Limit(POWER_LIMIT_FORM_SYSTEM), \
					 GM6020_YAW_Angle_To_360(gimbal_motor_feedback_parsed_data[*yaw_motor_index].mechanical_angle),\
					(float)robot_mode_data_pt->virtual_rocker.ch3 * CHASSIS_MOTOR_GYRO_BASE_RATE * chassis_motor_boost_rate, \
					(float)robot_mode_data_pt->virtual_rocker.ch2 * CHASSIS_MOTOR_GYRO_BASE_RATE * chassis_motor_boost_rate );
                }
                
			       case mk_chassis_special_mode_ENUM:///<����ģʽ
                {
                chassis_motor_speed[0] = -robot_mode_data_pt->virtual_rocker.ch3 - robot_mode_data_pt->virtual_rocker.ch2 + robot_mode_data_pt->virtual_rocker.ch0;
                chassis_motor_speed[1] = robot_mode_data_pt->virtual_rocker.ch3 - robot_mode_data_pt->virtual_rocker.ch2 + robot_mode_data_pt->virtual_rocker.ch0;
                chassis_motor_speed[2] = -robot_mode_data_pt->virtual_rocker.ch3 + robot_mode_data_pt->virtual_rocker.ch2 + robot_mode_data_pt->virtual_rocker.ch0;
                chassis_motor_speed[3] = robot_mode_data_pt->virtual_rocker.ch3 + robot_mode_data_pt->virtual_rocker.ch2 + robot_mode_data_pt->virtual_rocker.ch0;

                chassis_motor_speed[0] *= 10.0f;
                chassis_motor_speed[1] *= 10.0f;
                chassis_motor_speed[2] *= 10.0f;
                chassis_motor_speed[3] *= 10.0f;

                break;
            }
                     default:
            {
                break;
            }
        }
    }
			

    else if (robot_mode_data_pt-> mode.control_device == remote_controller_device_ENUM)///<ң��������
        {

        switch (robot_mode_data_pt->mode.rc_motion_mode)///<ѡ�������̨�Ĺ���ģʽ
        {

                case rc_chassis_follow_mode_ENUM: ///< 1�����̸���+�ֶ���׼
            {
                follow_pid_output = Calc_Chassis_Follow(); ///< ���̸��� pid,����� yaw �ᵱǰ�ǶȺ� yaw ��ͷ֮��ĽǶȲ�����ٶ�ֵ�������������

                chassis_motor_speed[0] = -rc_data_pt->rc.ch3 + rc_data_pt->rc.ch2 + follow_pid_output + rc_data_pt->rc.ch0 / 2.9f;
                chassis_motor_speed[1] = rc_data_pt->rc.ch3 + rc_data_pt->rc.ch2 + follow_pid_output + rc_data_pt->rc.ch0 / 2.9f;
                chassis_motor_speed[2] = -rc_data_pt->rc.ch3 - rc_data_pt->rc.ch2 + follow_pid_output + rc_data_pt->rc.ch0 / 2.9f;
                chassis_motor_speed[3] = rc_data_pt->rc.ch3 - rc_data_pt->rc.ch2 + follow_pid_output + rc_data_pt->rc.ch0 / 2.9f;

                chassis_motor_speed[0] *= motor_speed_multiple;
                chassis_motor_speed[1] *= motor_speed_multiple;
                chassis_motor_speed[2] *= motor_speed_multiple;
                chassis_motor_speed[3] *= motor_speed_multiple;
                break;
            }

            case rc_chassis_gyro_mode_ENUM: ///< 2������С����+�ֶ���׼
            {
                Calc_Gyro_Motors_Speed(chassis_motor_speed,
                                       0,
                                       GM6020_YAW_Angle_To_360(gimbal_motor_feedback_parsed_data[*yaw_motor_index].mechanical_angle),
                                       rc_data_pt->rc.ch3 * 8.0f,
                                       rc_data_pt->rc.ch2 * 8.0f);
                break;
            }

            case rc_special_mode_ENUM: ///< 3������ģʽ

            {
                chassis_motor_speed[0] = rc_data_pt->rc.ch3 - rc_data_pt->rc.ch2 + rc_data_pt->rc.ch0;
                chassis_motor_speed[1] = -rc_data_pt->rc.ch3 - rc_data_pt->rc.ch2 + rc_data_pt->rc.ch0;
                chassis_motor_speed[2] = rc_data_pt->rc.ch3 + rc_data_pt->rc.ch2 + rc_data_pt->rc.ch0;
                chassis_motor_speed[3] = -rc_data_pt->rc.ch3 + rc_data_pt->rc.ch2 + rc_data_pt->rc.ch0;

                chassis_motor_speed[0] *= motor_speed_multiple;
                chassis_motor_speed[1] *= motor_speed_multiple;
                chassis_motor_speed[2] *= motor_speed_multiple;
                chassis_motor_speed[3] *= motor_speed_multiple;

                break;
            }

            default:
            {
                break;
            }
        }
    }

//        OUTPUT_LIMIT(&chassis_motor_speed[0], 8899);
//        OUTPUT_LIMIT(&chassis_motor_speed[1], 8899);
//        OUTPUT_LIMIT(&chassis_motor_speed[2], 8899);
//        OUTPUT_LIMIT(&chassis_motor_speed[3], 8899);

	#if CHASSIS_SPEED_ZERO
			motor_speed[0] = 0;
			motor_speed[1] = 0;
			motor_speed[2] = 0;
			motor_speed[3] = 0;
		#endif
        ///< ���õ��̵���ٶ�
        Set_ChassisMotor_Speed(chassis_motor_speed[0],
                               chassis_motor_speed[1],
                               chassis_motor_speed[2],
                               chassis_motor_speed[3],
                               chassis_motor_feedback_parsed_data);

        osDelay(10);
    }
}
void Chassis_Init(void)
{
    rc_data_pt = Get_Rc_Parsed_RemoteData_Pointer();                              // ��ȡ�������ң��������
    robot_mode_data_pt = Get_Parsed_RobotMode_Pointer();                          //������ģʽ�ṹ��ָ��
    chassis_motor_feedback_parsed_data = Get_Can1_Feedback_Data();                //CAN1 �����ϵ���ķ�������
    gimbal_motor_feedback_parsed_data = Get_Gimbal_Parsed_FeedBack_Data();        //CAN2�����ϵ���ķ�������
    yaw_motor_index = Get_Yaw_Motor_Index();                                      // ��ȡ yaw �����������е��±�
    pitch_motor_index = Get_Pitch_Motor_Index();                                  //��ȡpitch�����������е��±�
    (void)pitch_motor_index;
}

/**
  * @brief          ����С����ʱ����������ٶ�
  * @details        Ч���������������ٶ�С������ת��������״̬��ͬʱ��������̨ǹ��Ϊͷ��ǰ�����ˡ�����ƽ��
  * @param[in]      {float *motors_speed }      �������ٶȵ�ָ��
  * @param[in]      {float rotate_speed }       ��ת�ĵ���ٶ�
  * @param[in]      {float move_direction }     �˶��ķ���Ƕ�0��~360�㣬�Ի�������Ϊ0��(��̨GM6020�����Ե��̵ĽǶ�)
  * @param[in]      {float x_move_speed}        x���ٶȣ�Xҡ�ˣ�
  * @param[in]      {float y_move_speed}        y���ٶȣ�Yҡ�ˣ�
  * @retval                                     void
  */
void Calc_Gyro_Motors_Speed(float *motors_speed, float rotate_speed, float move_direction, float x_move_speed, float y_move_speed)
{
    ///< �жϽǶ��Ƿ����
    if (move_direction > 360)
    {
        return;
    }

    ///< �ǶȻ��㻡��
    float move_radin = move_direction * 3.14159f / 180.0f;
    float radin_sin = sinf(move_radin); ///< sin(x)
    float radin_cos = cosf(move_radin);

    ///< �����ٶȸ�ֵ
    motors_speed[0] = rotate_speed;
    motors_speed[1] = rotate_speed;
    motors_speed[2] = rotate_speed;
    motors_speed[3] = rotate_speed;

    ///< �����ٶ�����ֵ
    float x_x_speed = x_move_speed * radin_cos;
    float x_y_speed = x_move_speed * radin_sin;

    float y_x_speed = y_move_speed * radin_sin;
    float y_y_speed = y_move_speed * radin_cos;

    motors_speed[0] += ((x_x_speed - x_y_speed) + (y_x_speed + y_y_speed));
    motors_speed[1] += ((-x_x_speed - x_y_speed) + (-y_x_speed + y_y_speed));
    motors_speed[2] += ((x_x_speed + x_y_speed) + (y_x_speed - y_y_speed));
    motors_speed[3] += ((-x_x_speed + x_y_speed) + (-y_x_speed - y_y_speed));
}
/**
 * @author          whale copy from bashpow
 * @brief           ͨ����ȡ����ϵͳ�����ƹ��ʼ���С����ʱ���������M3508�����ٶȣ�С�����ٶ�Ŀ��ֵ��
 * @details         ���ص��ǲ�ͬʵʱ�����µ�С���ݵ��̵���ٶ�Ŀ��ֵ
 * @note            ������ٷ���Դ���ʿ��ƱȽ�֮���޸�һ�£����о��ǹ��ʼ���Ҫ�ĳ�Ӣ�۵�
 * @param [in]      ͨ������ϵͳ��ȡ�Ĺ���
 * */
static uint16_t Calc_Gyro_Speed_By_Power_Limit(uint16_t power_limit)
{
	if(power_limit < 50)
	{
		return 1800;
	}
	else if(power_limit > 300)
	{
		return 2000;
	}
	else if(power_limit > 120)
	{
		return 6000;
	}

	return power_limit*60;
}

/**
 * @brief           ������̸��� pid
 * @param[in]       void
 * @return          float ��������ٶ�ֵ
 */
float Calc_Chassis_Follow(void)
{
#define YAW_INIT_ANGLE YAW_GM6020_HEAD_ANGLE

    float follow_tar = YAW_INIT_ANGLE;
    float follow_cur = gimbal_motor_feedback_parsed_data[*yaw_motor_index].mechanical_angle;

    Handle_Angle8191_PID_Over_Zero(&follow_tar, &follow_cur);

    return Pid_Position_Calc(&chassis_follow_pid, follow_tar, follow_cur);

#undef YAW_INIT_ANGLE
}
