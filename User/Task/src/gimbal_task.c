#include "gimbal_task.h"
#include "can2_device.h"
/**
 * @note    pitch_down    7880      up          850
 * DOWN 8000     MID 340   UP  700
 * @note    yaw  6088
  */
 /* ע��������λ������� */

static const uint8_t gimbal_motor_num = 2;           ///< ��̨���������
/* Yaw&Pitch����±� */
static const uint8_t yaw_motor_index = 0;            ///< yaw �����ڵ�����ݽṹ���е��±�
static const uint8_t pitch_motor_index = 1;          ///< pitch �����ڵ�����ݽṹ���е��±�
/* Pitch */
static const uint16_t pitch_up_angle_limit = 700;   ///< pitch ����̨��ͽǶ�
static const uint16_t pitch_middle_angle = 5000;     ///< pitch ����̨�м�Ƕ� 
static const uint16_t pitch_down_angle_limit = 8000; ///< pitch ����̨��߽Ƕ�  /* ��һ�β����Ƕ�90������ʹ��8090���� *
/* Yaw */
static float yaw_angle_set = 0 ;///< yaw ����̨���õĽǶȣ�TODO:��ʼ��ʱ���б�º�����
static float pitch_angle_set = pitch_middle_angle;   ///< pitch ����̨���õĽǶ�

static CAN_RxHeaderTypeDef *can2_rx_header_pt;       ///< can2 ���յ�ͷ���ݽṹ��ָ��
static uint8_t *can2_rxd_data_buffer;                ///< can2 ���յ����ݴ�ŵ������׵�ַ
static Rc_Ctrl_t *rc_data_pt;                        ///< ָ��ң�������ݵĽṹ��ָ��
static Robot_control_data_t *robot_mode_data_pt;     ///< ָ�������ģʽ�Ľṹ��ָ��
static Imu_t* imu_date_pt;                           ///< ָ�������ǻ�ȡ�ǶȵĽṹ��ָ��
static Motor_Measure_t gimbal_motor_parsed_feedback_data[gimbal_motor_num]; ///< ���������̨�����������(Yaw:0;Pitch:1)


/* pid����֮������ */
float pid_out[ALL_PID] = {0};///< �������Ҳ�ɷ���ÿ��task.c�У�����Ϊ��̬����
/* ��̨��ʼ���������� */
void Gimbal_Init(void);
void StartGimbalTask(void const *argument)
{
    // float yaw_target_speed, pitch_target_speed;//������ֲ�����飬����ԭ��������ʱע�Ͳ�ʹ��

// //�������򣬵��Խ���ɾ����ȫ��ע��
//     (void)pitch_down_angle_limit;   ///< ���Խ׶Σ����� warning
//     (void)pitch_up_angle_limit;

    ///< �����Խ׶�ʹ��
    // robot_mode_data_pt->mode.control_device = 2; 
    // robot_mode_data_pt->mode.rc_motion_mode = 5;///< �ṹ����ң����������ģʽ��Ȼ��5
//�����������

    Gimbal_Init();
    __OPEN_CAN2_RX_FIFO0_IT__;      ///<��CAN�����ж�
    /* ԭ������ʱ֮���ϳ�����֮���ֽ��������в���������ʱ��������ֲ���������֮����ʱ��ʧ */    
    osDelay(1000);
    /*bug_note ���ڴ�ӡ�������ֵʱ���г��̲�һ������ʱ���п�������й� */
    for (;;)
    {
		// Console.print("yaw %.2f, pitch %.2f, rol %.2f \r\n", imu_date_pt->yaw, imu_date_pt->pit, imu_date_pt->rol);
        ///< ���������̨��Yaw���������������ȶ������ȶ�
        //< ���еĵ��Դ�ӡȫ������debug-printf����usart3.h�еĺ궨������������룬�رպ�֮��debug_printfָ��պ�
        /* ������ݽ��� */
        Parse_Can2_Gimbal_Rxd_Data(can2_rx_header_pt, can2_rxd_data_buffer, gimbal_motor_parsed_feedback_data);
        
        /*ѡ������豸*/
        if (robot_mode_data_pt -> mode.control_device == remote_controller_device_ENUM)///<ң����ģʽ
        {
           // ������̨ģʽ 1���̸��� 2С����  3����
            switch (robot_mode_data_pt->mode.rc_motion_mode)
            {

            case 1:     ///<���̸���
            {   
                /*
                ���̸���ģʽ��С����ģʽ����̨�������߼����ã�����ڵ���
                */
            }
            
            case 2:     ///<����С����+��̨�����˶�

            {
                // Console.print("%0.2f,%0.2f\r\n",rc_data_pt->rc.ch0,rc_data_pt->rc.ch1);
                pitch_angle_set -= (rc_data_pt->rc.ch1) /6.0f;//��֮ǰ��*10.0f
                /*pitch���е�Ƕ��޷�*/
                Pitch_Angle_Limit(&pitch_angle_set, pitch_down_angle_limit, pitch_up_angle_limit);

                yaw_angle_set -= (rc_data_pt->rc.ch0) / 160.0f;
                // Console.print("%0.2f\r\n",pitch_angle_set);
                //yaw���趨ֵ�ǶȻػ�
                if(yaw_angle_set > 360)
                    {yaw_angle_set -= 360;}
				if(yaw_angle_set < 0) 
                    {yaw_angle_set += 360;}

//���������ݣ���Ư̫����仰���ջأ��ڼҲ����˼��Σ����������ǵ���Ư��ʵ���󣬿����Ǵ����Ʊ������ˡ�������ʹ�ùٷ������в��������ƽ����ĳ�����Խ��������⣩
                // yaw_target_speed = Calc_Yaw_Angle360_Pid(yaw_angle_set,imu_date_pt->yaw);
                /* ��һ���Ǳ�����ֲ */
                // Console.print("%0.2f\n",imu_date_pt->pit);
                pid_out[Yaw_target_Angle] = Calc_Yaw_Angle360_Pid(yaw_angle_set,imu_date_pt->yaw);

                // pitch_target_speed = Calc_Pitch_Angle8191_Pid(pitch_angle_set, &gimbal_motor_parsed_feedback_data[pitch_motor_index]);
                pid_out[Pitch_target_Speed] = Calc_Pitch_Angle8191_Pid(pitch_angle_set, &gimbal_motor_parsed_feedback_data[pitch_motor_index]);

             //debug_print("%.2f \r\n",pitch_target_speed);
            //     float new_moter_date1 = 0;
            //     new_moter_date1 =  gimbal_motor_parsed_feedback_data[pitch_motor_index].speed_rpm;
            //  Console.print("%0.2f ,%0.2f \r\n",pid_out[Yaw_target_Angle], new_moter_date1);
                break;
            }

            case 3:///<����ģʽ
            {
                pitch_angle_set -= (rc_data_pt->rc.ch1) / 6.0f;///< ԭ��*12
                /* bug,��δ������е�� */
				Pitch_Angle_Limit(&pitch_angle_set, pitch_up_angle_limit, pitch_down_angle_limit);///<pitch�Ƕ��޷�

                // yaw_target_speed = - rc_data_pt->rc.ch0 / 5.5f;
                pid_out[Yaw_target_Speed] = - (rc_data_pt->rc.ch0) / 5.5f;

				// pitch_target_speed = Calc_Pitch_Angle8191_Pid(pitch_angle_set,&gimbal_motor_parsed_feedback_data[pitch_motor_index]);
				pid_out[Pitch_target_Speed] = Calc_Pitch_Angle8191_Pid(pitch_angle_set,&gimbal_motor_parsed_feedback_data[pitch_motor_index]);
                
                break;
            }

            default:
            {

                break;
            }
            }
        }
        else if (robot_mode_data_pt->mode.control_device == mouse_keyboard_device_ENUM)///<����ģʽ
        {
            switch (robot_mode_data_pt->mode.mouse_keyboard_gimbal_mode)
            {
            case 1:///<�ֶ�ģʽ����̨����������������ƶ���
                {
                    /*Ŀ��Ƕ��趨*/
                    yaw_angle_set -= robot_mode_data_pt->virtual_rocker.ch0 / 58.0f;///<������Ҫ��������������ǲ�����
                    pitch_angle_set -= robot_mode_data_pt->virtual_rocker.ch1 / 1.6f;

                    //yaw�ǶȻػ�
					if(yaw_angle_set>360)
						{ yaw_angle_set -= 360;}
					if(yaw_angle_set<0) 
						{yaw_angle_set += 360;}
                    
                    /*pitch��Ƕ��޷�*/
                    Pitch_Angle_Limit(&pitch_angle_set,pitch_down_angle_limit,pitch_up_angle_limit);
                    /*���ᴮ��PID�ĽǶȻ�����*/
                    // pitch_target_speed  = Calc_Pitch_Angle8191_Pid(pitch_angle_set,&gimbal_motor_parsed_feedback_data[pitch_motor_index]);
                    pid_out[Pitch_target_Speed] = Calc_Pitch_Angle8191_Pid(pitch_angle_set,&gimbal_motor_parsed_feedback_data[pitch_motor_index]);

                    // yaw_target_speed    = Calc_Yaw_Angle360_Pid(yaw_angle_set,imu_date_pt->pit);
                    pid_out[Yaw_target_Speed] = Calc_Yaw_Angle360_Pid(yaw_angle_set,imu_date_pt->pit);
                }

                break;
                
            case 2:///<������̨ģʽ
            {
                /**Ŀǰ�Ĵ�������pitch��Ӿ��������ǣ��ǶȻ�ʵ�ʽǶ�ֵΪ�����Ƿ�����pitch���ݣ��趨ֵ��
                 * һ������ֵ��ֻ�ܱ��������ݸı䣨��ch0����,ch0��������ά����һ��ֵ���䣬�����ʹﵽ��������̨��Ч����pitch�ǲ������̻θ���
                 * ����ģʽ���˸о��ǳ���Ҫ
                 
                 */
                break;
            }
            case 3 :///<����ģʽ������̨����ϵΪ�����˶�����ϵ��ǰ�����ҵ��˶�������̨�ӽ�Ϊ׼����Chassis_task.c�о����˵����
            {
                pitch_angle_set += robot_mode_data_pt->virtual_rocker.ch1 /3.0f;
                /*û��yaw��Ƕ�����*/
                /*pitch�Ƕ�����*/
                Pitch_Angle_Limit(&pitch_angle_set,pitch_down_angle_limit,pitch_up_angle_limit);
                // yaw_target_speed    = robot_mode_data_pt->virtual_rocker.ch0 / 1.6f;
                pid_out[Yaw_target_Speed] = robot_mode_data_pt->virtual_rocker.ch0 / 1.6f;
                // pitch_target_speed  = Calc_Pitch_Angle8191_Pid(pitch_angle_set,&gimbal_motor_parsed_feedback_data[pitch_motor_index]);
                pid_out[Pitch_target_Speed] = Calc_Pitch_Angle8191_Pid(pitch_angle_set,&gimbal_motor_parsed_feedback_data[pitch_motor_index]);
            }
            default:
                break;
            }
        }
       // Console.print("%d,%d",pitch_target_speed,&gimbal_motor_parsed_feedback_data[pitch_motor_index])
       ///<��̨�����ٶȻ����㼰���͵��̵���ٶ�

    //    YAW_SPEED_OUTPUT_LIMIT(&yaw_target_speed,YAW_LIMIT_SPEED);  //yaw���ٶ�����(��ֲǰ)
        YAW_SPEED_OUTPUT_LIMIT(&pid_out[Yaw_target_Speed],YAW_LIMIT_SPEED);  //yaw���ٶ�����
       /* ֮ǰ���ٶ����ú��� */
        // Set_Gimbal_Motors_Speed(yaw_target_speed,
        //                         pitch_target_speed,
        //                         &gimbal_motor_parsed_feedback_data[yaw_motor_index],
        //                         &gimbal_motor_parsed_feedback_data[pitch_motor_index]);
        Set_Gimbal_Motors_Speed(pid_out[Yaw_target_Speed],
                                pid_out[Pitch_target_Speed],
                                &gimbal_motor_parsed_feedback_data[yaw_motor_index],
                                &gimbal_motor_parsed_feedback_data[pitch_motor_index]);
                            
/* ֱ��������в������ʹ�������ʹ���±����洢�������ֵ */
        // float new_moter_date = 0;
        // new_moter_date =  gimbal_motor_parsed_feedback_data[pitch_motor_index].mechanical_angle;
        // Console.print("%0.2f,%0.2f\r\n", new_moter_date,pid_out[Pitch_target_Speed]);
        osDelay(10);
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
    imu_date_pt = Get_Imu_Date_Now();                           ///<��ȡ�����ǽǶ�
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
