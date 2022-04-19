#include "gimbal_task.h"
#include "externel_gyroscope_task.h"
/*
pitch   ��ͷ    2520
        ��λ    3000
        ̧ͷ    3590
yaw     �Ƕ�    6440    Yaw�ǶȻ���ƫ�ƣ���֮��������Ҫÿ��ά��֮�����²�������
 */

/* shooter send id change to 0X1FF form 0x200 */
/* ��̨����±� */
static const uint8_t gimbal_motor_num = 2;  ///< ��̨���������
static const uint8_t yaw_motor_index = 0;   ///< yaw �����ڵ�����ݽṹ���е��±�
static const uint8_t pitch_motor_index = 1; ///< pitch �����ڵ�����ݽṹ���е��±�
/* Pitch�Ƕ��޷� */
static const uint16_t pitch_up_angle_limit = 3460;   ///< pitch ����̨��ͽǶ�
static const uint16_t pitch_middle_angle = 3000;     ///< pitch ����̨�м�Ƕ�
static const uint16_t pitch_down_angle_limit = 2600; ///< pitch ����̨��߽Ƕ�  ��һ�β����Ƕ�90������ʹ��8090����
/* Restart�Ƕ� */
static float yaw_angle_set = 0;                    ///< �����ʼ����0����Ϊ��chassis_task.c�У����ڸ�����ݳ�ʼ��е�ǶȽ����˴���
static float pitch_angle_set = pitch_middle_angle; ///< pitch ����̨���õĽǶ�
/* ָ��������� */
/* ��ֲ���жϽ��պ�Ӧɾ�����ֱ��� */
static CAN_RxHeaderTypeDef *can2_rx_header_pt;                              ///< can2 ���յ�ͷ���ݽṹ��ָ��
static uint8_t *can2_rxd_data_buffer;                                       ///< can2 ���յ����ݴ�ŵ������׵�ַ
static Rc_Ctrl_t *rc_data_pt;                                               ///< ָ��ң�������ݵĽṹ��ָ��
static Robot_control_data_t *robot_mode_data_pt;                            ///< ָ�������ģʽ�Ľṹ��ָ��
static Imu_t *imu_date_pt;                                                  ///< ָ�������ǻ�ȡ�ǶȵĽṹ��ָ��
static Motor_Measure_t gimbal_motor_parsed_feedback_data[gimbal_motor_num]; ///< ���������̨�����������(Yaw:0;Pitch:1)
static Wt61c_Data_t *imu_date_usart6;                                       ///< ָ�������Ĵ�������������

/* pid����֮������ */
float pid_out[ALL_PID] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; ///< �������Ҳ�ɷ���ÿ��task.c�У�����Ϊ��̬����
/* ��̨��ʼ���������� */
void Gimbal_Init(void);
/* ���ڵ��εı��� */
extern Pid_Position_t motor_pitch_speed_pid;

void StartGimbalTask(void const *argument)
{
    //��������
    /* ������ʱ������ʼ��ʱ��ģʽ */
    // robot_mode_data_pt->mode.control_device = 2;
    // robot_mode_data_pt->mode.rc_motion_mode = 4;///< �ṹ����ң����������ģʽ��Ȼ��5
    //�����������

    Gimbal_Init();

    osDelay(1000);
    for (;;)
    {
        ///< ���������̨��Yaw���������������ȶ������ȶ�
        //< ���еĵ��Դ�ӡȫ������debug.printf()����usart3.h�еĺ궨������������룬�رպ�֮��debug_printfָ��պ�
        /*ѡ������豸*/
        if (robot_mode_data_pt->mode.control_device == remote_controller_device_ENUM) ///<ң����ģʽ
        {
            // ������̨ģʽ: 1���̸��� 2С����  3������̨ 4����+С���� 5����ģʽ
            switch (robot_mode_data_pt->mode.rc_motion_mode)
            {

            case 1: ///<���̸���
            {
                /*���̸���ģʽ��С����ģʽ����̨�������߼�����
                ���������������Ƕȱջ���Ϊ�ٶ�ֵ������ڵ�������*/
            }

            case 2: ///<����С����+��̨�����˶�
            {
                pitch_angle_set -= (rc_data_pt->rc.ch1) / 12.0f; ///< ��֮ǰ��*10.0f
                /*pitch��Ƕ��޷�*/
                Pitch_Angle_Limit(&pitch_angle_set, pitch_down_angle_limit, pitch_up_angle_limit);
                yaw_angle_set = yaw_angle_set - ((rc_data_pt->rc.ch0) / 160.0f);
                // yaw���趨ֵ�ǶȻػ�
                if (yaw_angle_set > 360)
                {
                    yaw_angle_set -= 360;
                }
                if (yaw_angle_set < 0)
                {
                    yaw_angle_set += 360;
                }
                //���������ݣ���Ư̫��ʹ�ùٷ������в��������ƽ����ĳ�����ܿ��Խ���������
                /* Ŀǰʹ�����ô��������ǣ���ʱ������⡣����pitch������Ҳ��������ʹ�� */
                pid_out[Yaw_target_Speed] = Calc_Yaw_Angle360_Pid(yaw_angle_set, imu_date_usart6->angle.yaw_z);
                pid_out[Pitch_target_Speed] = Calc_Pitch_Angle8191_Pid(pitch_angle_set, &gimbal_motor_parsed_feedback_data[pitch_motor_index]);
                // debug_print("%0.2f,%0.2f\r\n", rc_data_pt->rc.ch0 * 1.0f, rc_data_pt->rc.ch1 * 1.0f);
                break;
            }
            case 3: ///< ����+��̨�����ƶ�
            {
                /* ������С��������̨�����߼� */
            }
            case 4: ///< ����+С����
            {

                pitch_angle_set -= rc_data_pt->rc.ch1 / 12.0f;
                yaw_angle_set -= (rc_data_pt->rc.ch0) / 160.0f; ///< ���ʴ�����
                /* �����������Ƕȱջ����û�е������λ */
                // Pitch_Angle_Limit(&pitch_angle_set, pitch_down_angle_limit, pitch_up_angle_limit);
                if (yaw_angle_set > 360)
                {
                    yaw_angle_set -= 360;
                }
                if (yaw_angle_set < 0)
                {
                    yaw_angle_set += 360;
                }

                pid_out[Yaw_target_Speed] = Calc_Yaw_Angle360_Pid(yaw_angle_set, imu_date_usart6->angle.yaw_z);
                pid_out[Pitch_target_Speed] = Calc_Pitch_Angle8191_Imu_Pid(pitch_angle_set, imu_date_pt);
                // Console.print("%0.2f,%0.2f,%0.2f\r\n", imu_date_pt->pit, pitch_angle_set, pid_out[Pitch_target_Speed]);
                /* bug_log:����ģʽ�³�����λֻ��ң�������Ƶ���Ӧ�����ƣ��������ǵ��µ�Ư�Ʋ����ܱ����� */
                /* Ŀǰ�Ľ������������װ��֮���������Ӧ�������ǽǶ�ֵ����������������0�㸽����΢С��Ư�Լ�������̨��Ư�� */
                break;
            }
            case 5: ///<����ģʽ
            {

                pitch_angle_set -= (rc_data_pt->rc.ch1) / 12.0f;
                // pid_out[Pitch_target_Speed] = rc_data_pt->rc.ch1/10.0f;///< �ٶȻ�����
                Pitch_Angle_Limit(&pitch_angle_set, pitch_up_angle_limit, pitch_down_angle_limit); ///< pitch�Ƕ��޷�
                /* С�����޷���ʹ��С�ڻ�е�޷�ֵ�İ�ȫ���� */
                /* ����������ģʽ��yaw�� */
                // pid_out[Yaw_target_Speed] = -(rc_data_pt->rc.ch0) / 1.0f;///ԭ�����ԵĲ�����Ӧ��Ŀ��ֵ
                pid_out[Yaw_target_Speed] = -(rc_data_pt->rc.ch0) / 6.0f;
                pid_out[Pitch_target_Speed] = Calc_Pitch_Angle8191_Pid(pitch_angle_set, &gimbal_motor_parsed_feedback_data[pitch_motor_index]);
                break;
            }

            default:
            {
                break;
            }
            }
        }
        else if (robot_mode_data_pt->mode.control_device == mouse_keyboard_device_ENUM) ///<����ģʽ
        {
            switch (robot_mode_data_pt->mode.mouse_keyboard_gimbal_mode)
            {
            case 1: ///<�ֶ�ģʽ����̨����������������ƶ���

            {
                /*Ŀ��Ƕ��趨*/
                yaw_angle_set -= robot_mode_data_pt->virtual_rocker.ch0 / 58.0f; ///<������Ҫ��������������ǲ�����
                pitch_angle_set -= robot_mode_data_pt->virtual_rocker.ch1 / 1.6f;

                // yaw�ǶȻػ�
                if (yaw_angle_set > 360)
                {
                    yaw_angle_set -= 360;
                }
                if (yaw_angle_set < 0)
                {
                    yaw_angle_set += 360;
                }
                /*pitch��Ƕ��޷�*/
                Pitch_Angle_Limit(&pitch_angle_set, pitch_down_angle_limit, pitch_up_angle_limit);

                /*���ᴮ��PID�ĽǶȻ�����*/
                pid_out[Pitch_target_Speed] = Calc_Pitch_Angle8191_Pid(pitch_angle_set, &gimbal_motor_parsed_feedback_data[pitch_motor_index]);
                // pid_out[Yaw_target_Speed] = Calc_Yaw_Angle360_Pid(yaw_angle_set, imu_date_usart6->angle.yaw_z);
            }

            break;

            case 2: ///<������̨ģʽ
            {
                /**Ŀǰ�Ĵ�������pitch��Ӿ��������ǣ��ǶȻ�ʵ�ʽǶ�ֵΪ�����Ƿ�����pitch���ݣ��趨ֵ��
                 * һ������ֵ��ֻ�ܱ��������ݸı䣨��ch0����,ch0��������ά����һ��ֵ���䣬�����ʹﵽ��������̨��Ч����pitch�ǲ������̻θ���
                 *ʹ��Ŀǰ��"-="���ԴﵽЧ��;
                 */
                /*Ŀ��Ƕ��趨*/
                yaw_angle_set -= robot_mode_data_pt->virtual_rocker.ch0 / 58.0f; ///<������Ҫ��������������ǲ�����
                pitch_angle_set -= robot_mode_data_pt->virtual_rocker.ch1 / 1.6f;

                // yaw�ǶȻػ�
                if (yaw_angle_set > 360)
                {
                    yaw_angle_set -= 360;
                }
                if (yaw_angle_set < 0)
                {
                    yaw_angle_set += 360;
                }
                /*pitch��Ƕ��޷�*/
                Pitch_Angle_Limit(&pitch_angle_set, pitch_down_angle_limit, pitch_up_angle_limit);

                /*���ᴮ��PID�ĽǶȻ�����*/
                pid_out[Pitch_target_Speed] = Calc_Pitch_Angle8191_Pid(pitch_angle_set, &gimbal_motor_parsed_feedback_data[pitch_motor_index]);
                /* ��һ������д�ǶȻ�PID���㺯����pitch��ʹ������������ */
                // pid_out[Yaw_target_Speed] = Calc_Yaw_Angle360_Pid(yaw_angle_set, imu_date_usart6->angle.yaw_z);
                break;
            }
            case 3: ///<����ģʽ������̨����ϵΪ�����˶�����ϵ��ǰ�����ҵ��˶�������̨�ӽ�Ϊ׼����Chassis_task.c�о����˵����
            {
                pitch_angle_set += robot_mode_data_pt->virtual_rocker.ch1 / 3.0f;
                /*pitch�Ƕ�����*/
                // Pitch_Angle_Limit(&pitch_angle_set,pitch_up_angle_limit, pitch_down_angle_limit);
                // pid_out[Yaw_target_Speed] = robot_mode_data_pt->virtual_rocker.ch0 / 1.6f;/
                pid_out[Pitch_target_Speed] = Calc_Pitch_Angle8191_Pid(pitch_angle_set, &gimbal_motor_parsed_feedback_data[pitch_motor_index]);
                break;
            }
            default:
                break;
            }
        }
        // Console.print("%0.2f,%0.2f,%0.2f\r\n", imu_date_pt->pit, pitch_angle_set, pid_out[Pitch_target_Speed]);
        ///<��̨�����ٶȻ����㼰���͵��̵���ٶ�

        YAW_SPEED_OUTPUT_LIMIT(&pid_out[Yaw_target_Speed], YAW_LIMIT_SPEED); // yaw���ٶ�����

        /**
         * @brief   ��̨�ٶȻ������Լ�CAN2���͵������
          */
        Set_Gimbal_Motors_Speed(
            pid_out[Yaw_target_Speed],
            // 0,
            pid_out[Pitch_target_Speed],
            gimbal_motor_parsed_feedback_data[yaw_motor_index].speed_rpm,
            gimbal_motor_parsed_feedback_data[pitch_motor_index].speed_rpm);

        /* ֱ��������в������ʹ�������ʹ���±����洢�������ֵ */
        float new_moter_speed = gimbal_motor_parsed_feedback_data[yaw_motor_index].speed_rpm;
        float new_moter_angle = gimbal_motor_parsed_feedback_data[yaw_motor_index].mechanical_angle;
        // Console.print("%0.2f\r\n", new_moter_mac); ///�����е�Ƕ�
        pitch_angle_set -= (rc_data_pt->rc.ch1) / 12.0f;
        // Console.print("%0.2f,%0.2f,%0.2f,%0.2f\r\n", new_moter_date, pid_out[Yaw_target_Speed]);
        Console.print("%0.2f,%0.2f,%0.2f\n", new_moter_angle, yaw_angle_set, rc_data_pt->rc.ch0);
        // Console.print("%0.2f,%0.2f\r\n", imu_date_usart6->angle.yaw_z, imu_date_usart6->angle.pitch_y);
        osDelay(5);
    }
}

/**
 * @brief               ��̨��ص����г�ʼ��
 *                      CAN2��ʼ�����������ݽ��ճ�ʼ�������������ݽ��ճ�ʼ��
 * @param [in]          void
 * @return              void
 */
void Gimbal_Init(void)
{
    Can2_Filter_Init();

    can2_rx_header_pt = Get_CAN2_Rx_Header();
    can2_rxd_data_buffer = Get_CAN2_Rxd_Buffer();
    rc_data_pt = Get_Rc_Parsed_RemoteData_Pointer();
    robot_mode_data_pt = Get_Parsed_RobotMode_Pointer();
    imu_date_pt = Get_Imu_Date_Now(); ///<��ȡ�����ǽǶ�
    imu_date_usart6 = Get_Gyroscope_Data_t();
    gimbal_motor_parsed_feedback_data[yaw_motor_index] = Get_Yaw_Data();
    gimbal_motor_parsed_feedback_data[pitch_motor_index] = Get_Pitch_Data();
    __OPEN_CAN2_RX_FIFO0_IT__; ///<��CAN�����ж�
    /* ԭ������ʱ֮���ϳ�����֮����CAN2���������в��������ӳ٣�������ֲ���������֮����ʱ��ʧ */
}

// /**
//  * @brief   ��װ�Թ��жϵ��ã��������жϵ���ʱ���������
//  * @note    ������Ҫ�ѱ�����ֲ��can2_device.c���ع�����
//   */
void Can2_Parse_For_Callback(void)
{
    Parse_Can2_Gimbal_Rxd_Data(can2_rx_header_pt, can2_rxd_data_buffer, gimbal_motor_parsed_feedback_data);
}
/**
 * @brief                       ���� CAN2 ������̨���յ���������̨�����ݲ����н���
 * @param p_can_rx_header       ָ����� CAN2 ����ͷ�Ľṹ��ָ��
 * @param data                  ���յ��� 8 Byte ����
 * @param motor                 �������ĵ���ṹ�������׵�ַ
 * @retval                      void
 */
void Parse_Can2_Gimbal_Rxd_Data(CAN_RxHeaderTypeDef *p_can_rx_header, uint8_t *data, Motor_Measure_t *motor)
{

    switch (p_can_rx_header->StdId)
    {
    case CAN_YAW_MOTOR_ID:
    {
        Calculate_Motor_Data(&gimbal_motor_parsed_feedback_data[yaw_motor_index], data);
        break;
    }

    case CAN_PITCH_MOTOR_ID:
    {
        Calculate_Motor_Data(&gimbal_motor_parsed_feedback_data[pitch_motor_index], data);
        break;
    }

    // case 0:
    // {
    //     // TODO ���� CAN2 �����ϵ����ݷ���������
    //     // ���������̨�����У�Ӧ���� can2_device.c �н��н��������������������н���
    //     break;
    // }
    default:
    {
        break;
    }
    }
}

/**
 * @brief                       ��ȡ���������̨�������
 * @param[in]                   void
 * @return {Motor_Measure_t*}   ���������̨������ݣ������׵�ַ��
 */
Motor_Measure_t *Get_Gimbal_Parsed_FeedBack_Data(void)
{
    return gimbal_motor_parsed_feedback_data;
}

/**
 * @brief                       ��ȡ pitch �����������е��±�
 * @param[in]                   void
 * @return {const_uint8_t*}     �±�
 */
const uint8_t *Get_Pitch_Motor_Index(void)
{
    return &pitch_motor_index;
}

/**
 * @brief                       ��ȡ yaw �����������е��±�
 * @param[in]                   void
 * @return {const_uint8_t*}     �±�
 */
const uint8_t *Get_Yaw_Motor_Index(void)
{
    return &yaw_motor_index;
}

/**
 * @brief                       �� gm6020 �ĵ��ת��ֵ����ɽǶ�
 * @param[in] {gm6020_angle}    gm6020 �ĵ��ת��ֵ
 * @return {float}              �Ƕ�ֵ
 */
float GM6020_YAW_Angle_To_360(uint16_t gm6020_angle)
{
#define ROBOT_HEAD_ANGLE YAW_GM6020_HEAD_ANGLE
    int16_t yaw_angle = gm6020_angle - ROBOT_HEAD_ANGLE;
    if (yaw_angle < 0)
    {
        yaw_angle += 8191;
    }
    return (float)(yaw_angle * 360 / 8191);
#undef ROBOT_HEAD_ANGLE
}
