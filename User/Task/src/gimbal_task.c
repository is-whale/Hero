#include "gimbal_task.h"
#include "math2.h"//包含在gimbal_task.h出现报错,暂时改到这里

static const uint8_t gimbal_motor_num = 2;           ///< 云台电机的数量
static const uint8_t yaw_motor_index = 0;            ///< yaw 轴电机在电机数据结构体中的下标
static const uint8_t pitch_motor_index = 1;          ///< pitch 轴电机在电机数据结构体中的下标
static const uint16_t pitch_up_angle_limit = 6960;   ///< pitch 轴云台最低角度
static const uint16_t pitch_middle_angle = 7490;     ///< pitch 轴云台中间角度
static const uint16_t pitch_down_angle_limit = 8010; ///< pitch 轴云台最高角度

static float yaw_angle_set = 0;                      ///< yaw 轴云台设置的角度
static float pitch_angle_set = pitch_middle_angle;   ///< pitch 轴云台设置的角度

static CAN_RxHeaderTypeDef *can2_rx_header_pt;       ///< can2 接收的头数据结构体指针
static uint8_t *can2_rxd_data_buffer;                ///< can2 接收的数据存放的数组首地址
static Rc_Ctrl_t *rc_data_pt;                        ///< 指向遥控器数据的结构体指针
static Robot_control_data_t *robot_mode_data_pt;     ///< 指向机器人模式的结构体指针
//static Imu_t* imu_date_pt;                         ///<指向陀螺仪获取角度的结构体指针
static Motor_Measure_t gimbal_motor_parsed_feedback_data[gimbal_motor_num]; ///< 解析后的云台电机数据数组

void StartGimbalTask(void const *argument)
{
    float yaw_speed, pitch_speed;

    Can2_Filter_Init();
    can2_rx_header_pt = Get_CAN2_Rx_Header();
    can2_rxd_data_buffer = Get_CAN2_Rxd_Buffer();
    rc_data_pt = Get_Rc_Parsed_RemoteData_Pointer();
    robot_mode_data_pt = Get_Parsed_RobotMode_Pointer();
//    imu_date_pt = Get_Imu_Date_Now();                           ///<获取陀螺仪角度

//调试区域，调试结束删除或全部注释
    (void)pitch_down_angle_limit;   ///< 测试阶段，避免 warning
    (void)pitch_up_angle_limit;

    ///< 仅测试阶段使用
    robot_mode_data_pt->mode.control_device = 2;
    robot_mode_data_pt->mode.rc_motion_mode = 1;
//调试区域结束

    osDelay(1000);
    __OPEN_CAN2_RX_FIFO0_IT__;
    for (;;)
    {
				//Console.print("yaw %.2f, pitch %.2f, rol %.2f \r\n", imu_date_pt->yaw, imu_date_pt->pit, imu_date_pt->rol);
        Parse_Can2_Gimbal_Rxd_Data(can2_rx_header_pt, can2_rxd_data_buffer, gimbal_motor_parsed_feedback_data);//解析任务，需要搬到接收中断里面
        //操作逻辑
        if (robot_mode_data_pt->mode.control_device == remote_controller_device_ENUM)///<选择机器人模式 
        {
           // 底盘云台模式 1底盘跟随 2小陀螺  3特殊
            switch (robot_mode_data_pt->mode.rc_motion_mode)
            {
            case 1:     ///<底盘跟随    暂未添加陀螺仪数据
            case 2:     ///<底盘小陀螺云台自由运动
            /*
            底盘跟随模式和小陀螺模式在云台任务中逻辑共用，差别在底盘
            */
            {
                                pitch_angle_set = (rc_data_pt->rc.ch1) * -10.0f;//步兵是106f
                //解析没有出错，但通道信息是反的，暂时添加负号解决问题。后面搞清楚是硬件问题还是数据使用(八成是用错了)
                yaw_angle_set = (rc_data_pt->rc.ch0) / 12.0f;

                //yaw轴设定值角度回环
                if(yaw_angle_set > 360)
                    {yaw_angle_set -= 360;}
				if(yaw_angle_set < 0) 
                    {yaw_angle_set += 360;}
                    
                (void)yaw_speed;
                //还未添加陀螺仪数据，零漂太大（这句话我收回，在家测试了几次，板载陀螺仪的零漂其实不大，可能是磁力计被干扰了。看博客使用官方结算中不带磁力计矫正的程序可以解决这个问题）
               // yaw_speed = Calc_Yaw_Angle360_Pid(yaw_angle_set,300);

                //pitch限幅机械角需要测量，限幅函数一直是隐式定义报错
                // Float_Constraion(&pitch_angle_set, pitch_down_angle_limit, pitch_up_angle_limit); ///< pitch 角度限幅
                //Pitch_Angle_Limit(&pitch_angle_set, pitch_down_angle_limit, pitch_up_angle_limit);//需要重新测量

                pitch_speed = Calc_Pitch_Angle8191_Pid(pitch_angle_set, &gimbal_motor_parsed_feedback_data[yaw_motor_index]);

             // debug_print("%.2f \r\n",pitch_speed);
             // Console.print("%0.2f ,%0.2f \r\n",pitch_speed,);
                break;
            }
            case 3:///<特殊模式
            {
                pitch_angle_set -= (rc_data_pt->rc.ch1) / 12.0f;

                /* Pitach角度限制 */
				// Float_Constrain(&pitch_angle_set, PITCH_UP_LIMIT, PITCH_DOWN_LIMIT);

                yaw_speed = - rc_data_pt->rc.ch0 / 5.5f;
				pitch_speed = Calc_Pitch_Angle8191_Pid(pitch_angle_set,&gimbal_motor_parsed_feedback_data[pitch_motor_index]);
                
                break;
            }

            default:
            {
                break;
            }
            }
        }
       // Console.print("%d,%d",pitch_speed,)
       
        Set_Gimbal_Motors_Speed(yaw_speed,
                                pitch_speed,
                                &gimbal_motor_parsed_feedback_data[yaw_motor_index],
                                &gimbal_motor_parsed_feedback_data[pitch_motor_index]);
        float new_moter_date = 0;
        new_moter_date =  gimbal_motor_parsed_feedback_data[yaw_motor_index].speed_rpm;
        Console.print("%0.2f,%0.2f\r\n",pitch_speed, new_moter_date);
        osDelay(10);
    }
}

/**
 * @brief                       解析 CAN2 解析云台接收到的两个云台的数据并进行解析
 * @param p_can_rx_header       指向接收 CAN2 数据头的结构体指针
 * @param data                  接收到的 8 Byte 数据
 * @param motor                 传进来的电机结构体数组首地址
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
        // TODO 其他 CAN2 总线上的数据反馈解析，
        // 如果不在云台任务中，应当在 can2_device.c 中进行解析，或者在其他任务中解析
        break;
    }
    default:
    {
        break;
    }
    }
}

/**
 * @brief                       获取解析后的云台电机数据
 * @param[in]                   void
 * @return {Motor_Measure_t*}   解析后的云台电机数据
 */
Motor_Measure_t *Get_Gimbal_Parsed_FeedBack_Data(void)
{
    return gimbal_motor_parsed_feedback_data;
}

/**
 * @brief                       获取 pitch 轴电机在数组中的下标
 * @param[in]                   void
 * @return {const_uint8_t*}     下标
 */
const uint8_t *Get_Pitch_Motor_Index(void)
{
    return &pitch_motor_index;
}

/**
 * @brief                       获取 yaw 轴电机在数组中的下标
 * @param[in]                   void
 * @return {const_uint8_t*}     下标
 */
const uint8_t *Get_Yaw_Motor_Index(void)
{
    return &yaw_motor_index;
}

/**
 * @brief                       将 gm6020 的电机转子值换算成角度 
 * @param[in] {gm6020_angle}    gm6020 的电机转子值
 * @return {float}              角度值 
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
