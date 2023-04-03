/**
 * @file chassis_task.c
 * @brief ��������
 * @version 0.1
 * @date 2021-9-28
 * @copyright Copyright (c) 2021
 */
#include "chassis_task.h"
/* �궨�� */
#define OUTPUT_LIMIT(data, limit) Float_Constraion(data, -limit, limit) ///< ����޷�
/* �ٶȱ��� */
#define CHASSIS_MOTOR_DEFAULT_BASE_RATE 5.5f    ///< ����Ĭ���ٶȵı���
#define CHASSIS_MOTOR_GYRO_BASE_RATE 5.0f       ///< С���ݵ��ٶȱ���
static const float motor_speed_multiple = 13.5; ///< ����ٶȱ���
/* power */
#define POWER_LIMIT 60.0f
#define WARNING_POWER 55.0f
#define WARNING_POWER_BUFF 60.0f

#define NO_JUDGE_TOTAL_CURRENT_LIMIT 64000.0f //16000 * 4,
#define BUFFER_TOTAL_CURRENT_LIMIT 16000.0f
#define POWER_TOTAL_CURRENT_LIMIT 20000.0f
/* �޷� */
static float chassis_motor_boost_rate = 4.0f; ///< ���̵������
/* PID����ʵ���� */
static Pid_Position_t chassis_follow_pid = NEW_POSITION_PID(0.26, 0, 0.8, 5000, 500, 0, 1000, 500); ///< ���̸���PID
/* ����ָ�� */
static Rc_Ctrl_t *rc_data_pt;                               ///< ָ��������ң�����ṹ��ָ��
static Robot_control_data_t *robot_mode_data_pt;            ///< ָ�������Ļ�����ģʽ�ṹ��ָ��(Ҳ�������������ͨ����ֵ)
static Motor_Measure_t *chassis_motor_feedback_parsed_data; ///< ָ�������ĵ��̵������
static Motor_Measure_t *gimbal_motor_feedback_parsed_data;  ///< ָ����������̨�������
static const uint8_t *yaw_motor_index;                      ///< ָ��yaw ��������̨��������е��±�
static const Judge_data_t *referee_date_pt;                 ///< ָ�������Ĳ���ϵͳ����

/* ��ֲ��CAN1���� */
static const uint8_t can1_motor_device_number = 4;
static uint8_t *can1_rxd_data;
static CAN_RxHeaderTypeDef *can1_rx_header;
static uint8_t *can1_rxd_data;
static Motor_Measure_t m3508_feddback_data[can1_motor_device_number]; ///<����Ϊ������ĵ�����ݽṹ�壬���ΪCAN1������1~4

/*��������*/
void Chassis_Init(void);                                              ///<���̳�ʼ����������
static uint16_t Calc_Gyro_Speed_By_Power_Limit(uint16_t power_limit); ///<���㹦�������µ�С���ݻ��ߵ��̸���ʱ�ĵ���ٶ�Ŀ��ֵ
void Calc_Gyro_Motors_Speed(float *motors_speed, float rotate_speed,
                            float move_direction, float x_move_speed, float y_move_speed); ///< ����С����ʱ�ĵ���ٶ�

void StartChassisTask(void const *argument)
{
    static float chassis_motor_speed[4] = {0.0, 0.0, 0.0, 0.0}; ///< �����͵ĵ���ٶ�ֵ
    float follow_pid_output;                                    ///< ����PID���

    Chassis_Init(); ///<���̳�ʼ��

    osDelay(1000);

    for (;;)
    {
        HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
        ///<ѡ������豸    ����&ң����
        if (robot_mode_data_pt->mode.control_device == mouse_keyboard_device_ENUM) ///< ����ģʽ
        {
            /**
             * ѡ�������̨ģʽ:    1���̸���   2С����   3����ģʽ
             *
             * ����ģʽ����ʹ�����������ݣ��Ե�ǰ��̨����ϵ��Ϊ��������ϵ����ǰ����������̨�ӽ�Ϊ׼
             * ����ģʽ������������ʧЧʱ���õ�ģʽ��Ҳ���ϵ�֮��ĳ�ʼģʽ
             */
            switch (robot_mode_data_pt->mode.mouse_keyboard_chassis_mode)
            {
            case mk_chassis_follow_mode_ENUM: ///<���̸���
            {
                follow_pid_output = Calc_Chassis_Follow(); ///< �������PID���

                chassis_motor_speed[0] = robot_mode_data_pt->virtual_rocker.ch2 + robot_mode_data_pt->virtual_rocker.ch3 + follow_pid_output + rc_data_pt->mouse.x / 0.38f;
                chassis_motor_speed[1] = robot_mode_data_pt->virtual_rocker.ch2 - robot_mode_data_pt->virtual_rocker.ch3 + follow_pid_output + rc_data_pt->mouse.x / 0.38f;
                chassis_motor_speed[2] = -robot_mode_data_pt->virtual_rocker.ch2 + robot_mode_data_pt->virtual_rocker.ch3 + follow_pid_output + rc_data_pt->mouse.x / 0.38f;
                chassis_motor_speed[3] = -robot_mode_data_pt->virtual_rocker.ch2 - robot_mode_data_pt->virtual_rocker.ch3 + follow_pid_output + rc_data_pt->mouse.x / 0.38f;
                ///< ��һ������
                chassis_motor_speed[0] *= (float)(CHASSIS_MOTOR_DEFAULT_BASE_RATE * chassis_motor_boost_rate);
                chassis_motor_speed[1] *= (float)(CHASSIS_MOTOR_DEFAULT_BASE_RATE * chassis_motor_boost_rate);
                chassis_motor_speed[2] *= (float)(CHASSIS_MOTOR_DEFAULT_BASE_RATE * chassis_motor_boost_rate);
                chassis_motor_speed[3] *= (float)(CHASSIS_MOTOR_DEFAULT_BASE_RATE * chassis_motor_boost_rate);
            }

            case mk_chassis_gyro_mode_ENUM: ///<����С����

            {
                ///< ����С����ʱ�ĵ���ٶ�
                Calc_Gyro_Motors_Speed(chassis_motor_speed,
                                       Calc_Gyro_Speed_By_Power_Limit(referee_date_pt->power_heat_data.chassis_power),
                                       GM6020_YAW_Angle_To_360(gimbal_motor_feedback_parsed_data[*yaw_motor_index].mechanical_angle),
                                       (float)robot_mode_data_pt->virtual_rocker.ch3 * CHASSIS_MOTOR_GYRO_BASE_RATE * chassis_motor_boost_rate,
                                       (float)robot_mode_data_pt->virtual_rocker.ch2 * CHASSIS_MOTOR_GYRO_BASE_RATE * chassis_motor_boost_rate);
            }

            case mk_chassis_special_mode_ENUM: ///<����ģʽ
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

        else if (robot_mode_data_pt->mode.control_device == remote_controller_device_ENUM) ///<ң��������
        {

            switch (robot_mode_data_pt->mode.rc_motion_mode) ///<ѡ�������̨ģʽ
            {
            case rc_stable_chassis_follow_mode_ENUM: ///< 3:���̸���+������̨
            {
                /* ���̸����߼����� */
            }
            case rc_chassis_follow_mode_ENUM: ///< 1�����̸���+�ֶ���̨
            {
                follow_pid_output = Calc_Chassis_Follow(); ///< ���̸���pid

                chassis_motor_speed[0] = rc_data_pt->rc.ch3 + rc_data_pt->rc.ch2 + follow_pid_output + rc_data_pt->rc.ch0 / 2.9f;
                chassis_motor_speed[1] = -rc_data_pt->rc.ch3 + rc_data_pt->rc.ch2 + follow_pid_output + rc_data_pt->rc.ch0 / 2.9f;
                chassis_motor_speed[2] = rc_data_pt->rc.ch3 - rc_data_pt->rc.ch2 + follow_pid_output + rc_data_pt->rc.ch0 / 2.9f;
                chassis_motor_speed[3] = -rc_data_pt->rc.ch3 - rc_data_pt->rc.ch2 + follow_pid_output + rc_data_pt->rc.ch0 / 2.9f;

                chassis_motor_speed[0] *= motor_speed_multiple;
                chassis_motor_speed[1] *= motor_speed_multiple;
                chassis_motor_speed[2] *= motor_speed_multiple;
                chassis_motor_speed[3] *= motor_speed_multiple;
                break;
            }

            case rc_chassis_gyro_mode_ENUM: ///< 2������С����+�ֶ���׼
            {
                /* С���ݵ����߼����� */
            }
            case rc_stable_chassis_gyro_mode_ENUM: ///< 4������С����+������̨
            {
                Calc_Gyro_Motors_Speed(chassis_motor_speed,
                                       Calc_Gyro_Speed_By_Power_Limit(referee_date_pt->power_heat_data.chassis_power),
                                       GM6020_YAW_Angle_To_360(gimbal_motor_feedback_parsed_data[*yaw_motor_index].mechanical_angle),
                                       rc_data_pt->rc.ch3 * 8.0f,
                                       rc_data_pt->rc.ch2 * 8.0f);
                break;
            }

            case rc_special_mode_ENUM: ///< 5������ģʽ

            {
                Calc_Gyro_Motors_Speed(chassis_motor_speed,
                                       0,
                                       GM6020_YAW_Angle_To_360(gimbal_motor_feedback_parsed_data[*yaw_motor_index].mechanical_angle),
                                       rc_data_pt->rc.ch3 * 10.0f,
                                       rc_data_pt->rc.ch2 * 10.0f);

                break;
            }

            default:
            {
                break;
            }
            }
        }
        /* �ת�� 469rpm,��������ٱ�ԼΪ19:1 */
        /* ���Խ׶��ٶ��޷���8899�ĳ�889 */
        OUTPUT_LIMIT(&chassis_motor_speed[0], 8899);
        OUTPUT_LIMIT(&chassis_motor_speed[1], 8899);
        OUTPUT_LIMIT(&chassis_motor_speed[2], 8899);
        OUTPUT_LIMIT(&chassis_motor_speed[3], 8899);

//���̹رպ궨�壬��config.h
#if CHASSIS_SPEED_ZERO
        chassis_motor_speed[0] = 0;
        chassis_motor_speed[1] = 0;
        chassis_motor_speed[2] = 0;
        chassis_motor_speed[3] = 0;
#endif
        /**
 * @brief   �����ٶ�PID�������Լ����õ����ٶ�
  */
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
    /* ���� */
    can1_rxd_data = Get_CAN1_Rxd_Data();   ///< ����CAN1�յ���ԭʼ����
    can1_rx_header = Get_CAN1_Rx_Header(); ///< ���� CAN1 ��������ͷ�ṹ��ָ��

    Can1_Filter_Init();                                                    ///< CAN1�ײ��ʼ��
    referee_date_pt = Get_Referee_Data();                                  ///< ������Ĳ���ϵͳ����
    rc_data_pt = Get_Rc_Parsed_RemoteData_Pointer();                       // ��ȡ�������ң��������
    robot_mode_data_pt = Get_Parsed_RobotMode_Pointer();                   //������ģʽ�ṹ��ָ��
    chassis_motor_feedback_parsed_data = Get_Can1_Feedback_Data();         // CAN1 �����ϵ���ķ�������
    gimbal_motor_feedback_parsed_data = Get_Gimbal_Parsed_FeedBack_Data(); // CAN2�����ϵ���ķ�������
    yaw_motor_index = Get_Yaw_Motor_Index();                               // ��ȡ yaw �����������е��±�
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
 * @author          bashpow
 * @brief           С���ݵ������ƹ���
 * @details         ���ص��ǲ�ͬʵʱ�����µ�С���ݵ��̵���ٶ�Ŀ��ֵ
 * @note            ������ٷ���Դ���ʿ��ƱȽ�֮���޸�һ�£����о��ǹ��ʼ���Ҫ�ĳ�Ӣ�۵�
 * @param [in]      ͨ������ϵͳ��ȡ�Ĺ���
 * */
static uint16_t Calc_Gyro_Speed_By_Power_Limit(uint16_t power_limit)
{
    if (power_limit < 50)
    {
        return 1800;
    }
    else if (power_limit > 300)
    {
        return 2000;
    }
    else if (power_limit > 120)
    {
        return 6000;
    }

    return power_limit * 60;
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

    return -(Pid_Position_Calc(&chassis_follow_pid, follow_tar, follow_cur));

#undef YAW_INIT_ANGLE
}

/**
 * @brief           ���� CAN1 �ϵķ�������
 * @param[in]       void
 * @retval          void
 */
void Parse_Can1_Rxd_Data(void)
{
    static uint8_t i = 0;
    i = can1_rx_header->StdId - CAN_3508_M1_ID;
    Calculate_Motor_Data(&m3508_feddback_data[i], can1_rxd_data);
}

/**
 * @brief                       ���� CAN1 �����ϵ���ķ�������
 * @param[in]                   void
 * @return {Motor_Measure_t*}   ����������ݽṹ��ָ��
 */
Motor_Measure_t *Get_Can1_Feedback_Data(void)
{
    return m3508_feddback_data;
}
//���ü�ȡ��ַ������������ָ�������Ƕ���Ϊ�ṹ�塣��remote_task.c�в�ͬ��
/**
 * @brief                     ���� CAN1 �����ϵ��������
 * @param[in]                 void
 * @return {const uint8_t*}   CAN1 �����ϵ������
 */
const uint8_t *Get_Can1_Motor_DeviceNumber(void)
{
    return (&can1_motor_device_number);
}
/**
 * @brief           ֪ͨ CAN1 ���ݽ�������������ݽ���
 * Ŀǰ�ǽ���CAN1�ĸ����̵������
 * @param[in]       void
 * @retval          void
 * @note            ��ֲ�����Ͳ���Ҫ��
 */
void Info_Can1_ParseData_Task(void)
{
    Parse_Can1_Rxd_Data();
}

/**
  * @brief          ���ƹ��ʣ���Ҫ���Ƶ������
  * @param[in]      chassis_power_control: ��������
  * @retval         none
  */
void chassis_power_control(int16_t *chassis_motor1, int16_t *chassis_motor2, int16_t *chassis_motor3, int16_t *chassis_motor4)
{
    fp32 chassis_power = 0.0f;
    fp32 chassis_power_buffer = 0.0f;
    fp32 total_current_limit = 0.0f;
    fp32 total_current = 0.0f;

    chassis_power = referee_date_pt->power_heat_data.chassis_power;
    chassis_power_buffer = referee_date_pt->power_heat_data.chassis_power_buffer;
    // power > 80w and buffer < 60j, because buffer < 60 means power has been more than 80w
    //���ʳ���80w �ͻ�������С��60j,��Ϊ��������С��60��ζ�Ź��ʳ���80w
    if (chassis_power_buffer < WARNING_POWER_BUFF)
    {
        fp32 power_scale;
        if (chassis_power_buffer > 5.0f)
        {
            //��СWARNING_POWER_BUFF
            power_scale = chassis_power_buffer / WARNING_POWER_BUFF;
        }
        else
        {
            //only left 10% of WARNING_POWER_BUFF
            power_scale = 5.0f / WARNING_POWER_BUFF;
        }
        //��С
        total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT * power_scale;
    }
    else
    {
        //���ʴ���WARNING_POWER
        if (chassis_power > WARNING_POWER)
        {
            fp32 power_scale;
            //����С��80w
            if (chassis_power < POWER_LIMIT)
            {
                //��С
                power_scale = (POWER_LIMIT - chassis_power) / (POWER_LIMIT - WARNING_POWER);
            }
            //���ʴ���80w
            else
            {
                power_scale = 0.0f;
            }

            total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT * power_scale;
        }
        //����С��WARNING_POWER
        else
        {
            total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT;
        }
        // }
    }

    total_current = 0.0f;
    //����ԭ����������趨
    total_current += fabs(*chassis_motor1) + fabs(*chassis_motor2) + fabs(*chassis_motor3) + fabs(*chassis_motor4);

    if (total_current > total_current_limit)
    {
        fp32 current_scale = total_current_limit / total_current;
        *chassis_motor1 *= current_scale;
        *chassis_motor2 *= current_scale;
        *chassis_motor3 *= current_scale;
        *chassis_motor4 *= current_scale;
    }
}
