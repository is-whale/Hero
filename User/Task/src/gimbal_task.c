#include "gimbal_task.h"
#include "imu_mpu6050.h"

static const uint8_t gimbal_motor_num = 2;           ///< ��̨���������
static const uint8_t yaw_motor_index = 0;            ///< yaw �����ڵ�����ݽṹ���е��±�
static const uint8_t pitch_motor_index = 1;          ///< pitch �����ڵ�����ݽṹ���е��±�
static const uint16_t pitch_up_angle_limit = 6960;   ///< pitch ����̨��ͽǶ�
static const uint16_t pitch_middle_angle = 7490;     ///< pitch ����̨�м�Ƕ�
static const uint16_t pitch_down_angle_limit = 8010; ///< pitch ����̨��߽Ƕ�

static float yaw_angle_set = 0;                    ///< yaw ����̨���õĽǶ�
static float pitch_angle_set = pitch_middle_angle; ///< pitch ����̨���õĽǶ�

static CAN_RxHeaderTypeDef *can2_rx_header_pt;   ///< can2 ���յ�ͷ���ݽṹ��ָ��
static uint8_t *can2_rxd_data_buffer;            ///< can2 ���յ����ݴ�ŵ������׵�ַ
static Rc_Ctrl_t *rc_data_pt;                    ///< ָ��ң�������ݵĽṹ��ָ��
static Robot_control_data_t *robot_mode_data_pt; ///< ָ�������ģʽ�Ľṹ��ָ��

static Motor_Measure_t gimbal_motor_parsed_feedback_data[gimbal_motor_num]; ///< ���������̨�����������

void StartGimbalTask(void const *argument)
{
    float yaw_speed, pitch_speed;
    Can2_Filter_Init();
    can2_rx_header_pt = Get_CAN2_Rx_Header();
    can2_rxd_data_buffer = Get_CAN2_Rxd_Buffer();
    rc_data_pt = Get_Rc_Parsed_RemoteData_Pointer();
    robot_mode_data_pt = Get_Parsed_RobotMode_Pointer();

    //��ӻ�ȡ�����ǽǶ�

    (void)pitch_down_angle_limit;   ///< ���Խ׶Σ����� warning
    (void)pitch_up_angle_limit;
    (void)yaw_speed;

    ///< �����Խ׶�ʹ��
    robot_mode_data_pt->mode.control_device = 0;
    robot_mode_data_pt->mode.rc_motion_mode = 1;

    osDelay(1000);
    __OPEN_CAN2_RX_FIFO0_IT__;
/*
�����yaw�����
*/
    for (;;)
    {
        Parse_Can2_Gimbal_Rxd_Data(can2_rx_header_pt, can2_rxd_data_buffer, gimbal_motor_parsed_feedback_data);
        if (robot_mode_data_pt->mode.control_device == remote_controller_device_ENUM)///<ѡ�������ģʽ 
        {
           // ������̨ģʽ 1���̸��� 2С���� 3���� ���̸��� 4����С���� 5����
            switch (1)
            {
            case 1:     ///<���̸���

            {
                pitch_angle_set = (rc_data_pt->rc.ch1) * -10.0;//������106f
                //����û�г�����ͨ����Ϣ�Ƿ��ģ���ʱ��Ӹ���
                yaw_angle_set = (rc_data_pt->rc.ch0) / 12.0f;

                //yaw���趨ֵ�ǶȻػ�
                if(yaw_angle_set > 360) yaw_angle_set -= 360;
					if(yaw_angle_set < 0) yaw_angle_set += 360;

                (void)yaw_angle_set; ///< avoid warning
                
      //          yaw_speed = Calc_Yaw_Angle360_Pid(yaw_angle_set, 0);
                // Float_Constraion(&pitch_angle_set, pitch_down_angle_limit, pitch_up_angle_limit); ///< pitch �Ƕ��޷�
                //Pitch_Angle_Limit(&pitch_angle_set, pitch_down_angle_limit, pitch_up_angle_limit);//��Ҫ���²���

                pitch_speed = Calc_Pitch_Angle8191_Pid(pitch_angle_set, &gimbal_motor_parsed_feedback_data[pitch_motor_index]);

             // pitch_speed = (rc_data_pt->rc.ch0);
             // debug_print("%.2f \r\n",pitch_speed);
             // Console.print("%0.2f ,%0.2f \r\n",pitch_speed,);
           //  Console.print("%0.2f\r\n",1);
                break;
            }
            //gimbal_motor_parsed_feedback_data[pitch_motor_index].speed_rpm

            default:
            {
                break;
            }
            }
        }
       // Console.print("%d,%d",pitch_speed,)
        Set_Gimbal_Motors_Speed(0,
                                pitch_speed,
                                &gimbal_motor_parsed_feedback_data[yaw_motor_index],
                                &gimbal_motor_parsed_feedback_data[pitch_motor_index]);
        osDelay(10);
    }
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

    case 0:
    {
        // TODO ���� CAN2 �����ϵ����ݷ���������
        // ���������̨�����У�Ӧ���� can2_device.c �н��н��������������������н���
        break;
    }
    default:
    {
        break;
    }
    }
}

/**
 * @brief                       ��ȡ���������̨�������
 * @param[in]                   void
 * @return {Motor_Measure_t*}   ���������̨�������
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
