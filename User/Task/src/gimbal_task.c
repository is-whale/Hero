#include "gimbal_task.h"
#include "can2_device.h"
/**
 * @note    pitch_down    7880      up          850
 * DOWN 8000     MID 340   UP  700
 * @note    yaw  6088
  */
 /* 注意上下限位反的情况 */

static const uint8_t gimbal_motor_num = 2;           ///< 云台电机的数量
/* Yaw&Pitch电机下标 */
static const uint8_t yaw_motor_index = 0;            ///< yaw 轴电机在电机数据结构体中的下标
static const uint8_t pitch_motor_index = 1;          ///< pitch 轴电机在电机数据结构体中的下标
/* Pitch */
static const uint16_t pitch_up_angle_limit = 700;   ///< pitch 轴云台最低角度
static const uint16_t pitch_middle_angle = 5000;     ///< pitch 轴云台中间角度 
static const uint16_t pitch_down_angle_limit = 8000; ///< pitch 轴云台最高角度  /* 第一次测量角度90，后面使用8090正常 *
/* Yaw */
static float yaw_angle_set = 0 ;///< yaw 轴云台设置的角度（TODO:初始化时添加斜坡函数）
static float pitch_angle_set = pitch_middle_angle;   ///< pitch 轴云台设置的角度

static CAN_RxHeaderTypeDef *can2_rx_header_pt;       ///< can2 接收的头数据结构体指针
static uint8_t *can2_rxd_data_buffer;                ///< can2 接收的数据存放的数组首地址
static Rc_Ctrl_t *rc_data_pt;                        ///< 指向遥控器数据的结构体指针
static Robot_control_data_t *robot_mode_data_pt;     ///< 指向机器人模式的结构体指针
static Imu_t* imu_date_pt;                           ///< 指向陀螺仪获取角度的结构体指针
static Motor_Measure_t gimbal_motor_parsed_feedback_data[gimbal_motor_num]; ///< 解析后的云台电机数据数组(Yaw:0;Pitch:1)


/* pid计算之后的输出 */
float pid_out[ALL_PID] = {0};///< 这个参数也可放在每个task.c中，定义为静态变量
/* 云台初始化函数声明 */
void Gimbal_Init(void);
void StartGimbalTask(void const *argument)
{
    // float yaw_target_speed, pitch_target_speed;//数据移植到数组，所以原来参数暂时注释不使用

// //调试区域，调试结束删除或全部注释
//     (void)pitch_down_angle_limit;   ///< 测试阶段，避免 warning
//     (void)pitch_up_angle_limit;

    ///< 仅测试阶段使用
    // robot_mode_data_pt->mode.control_device = 2; 
    // robot_mode_data_pt->mode.rc_motion_mode = 5;///< 结构体内遥控器的特殊模式依然是5
//调试区域结束

    Gimbal_Init();
    __OPEN_CAN2_RX_FIFO0_IT__;      ///<打开CAN接收中断
    /* 原本在延时之后，上车测试之后发现解析数据有不定长的延时，所以移植到这里，测试之后延时消失 */    
    osDelay(1000);
    /*bug_note 串口打印电机反馈值时，有长短不一定的延时，有可能与此有关 */
    for (;;)
    {
		// Console.print("yaw %.2f, pitch %.2f, rol %.2f \r\n", imu_date_pt->yaw, imu_date_pt->pit, imu_date_pt->rol);
        ///< 板放置上云台除Yaw轴其他两轴数据稳定数据稳定
        //< 所有的调试打印全部采用debug-printf，在usart3.h中的宏定义控制条件编译，关闭宏之后debug_printf指向空宏
        /* 电机数据解析 */
        Parse_Can2_Gimbal_Rxd_Data(can2_rx_header_pt, can2_rxd_data_buffer, gimbal_motor_parsed_feedback_data);
        
        /*选择操作设备*/
        if (robot_mode_data_pt -> mode.control_device == remote_controller_device_ENUM)///<遥控器模式
        {
           // 底盘云台模式 1底盘跟随 2小陀螺  3特殊
            switch (robot_mode_data_pt->mode.rc_motion_mode)
            {

            case 1:     ///<底盘跟随
            {   
                /*
                底盘跟随模式和小陀螺模式在云台任务中逻辑共用，差别在底盘
                */
            }
            
            case 2:     ///<底盘小陀螺+云台自由运动

            {
                // Console.print("%0.2f,%0.2f\r\n",rc_data_pt->rc.ch0,rc_data_pt->rc.ch1);
                pitch_angle_set -= (rc_data_pt->rc.ch1) /6.0f;//改之前是*10.0f
                /*pitch轴机械角度限幅*/
                Pitch_Angle_Limit(&pitch_angle_set, pitch_down_angle_limit, pitch_up_angle_limit);

                yaw_angle_set -= (rc_data_pt->rc.ch0) / 160.0f;
                // Console.print("%0.2f\r\n",pitch_angle_set);
                //yaw轴设定值角度回环
                if(yaw_angle_set > 360)
                    {yaw_angle_set -= 360;}
				if(yaw_angle_set < 0) 
                    {yaw_angle_set += 360;}

//陀螺仪数据，零漂太大（这句话我收回，在家测试了几次，板载陀螺仪的零漂其实不大，可能是磁力计被干扰了。看博客使用官方结算中不带磁力计矫正的程序可以解决这个问题）
                // yaw_target_speed = Calc_Yaw_Angle360_Pid(yaw_angle_set,imu_date_pt->yaw);
                /* 下一行是变量移植 */
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

            case 3:///<特殊模式
            {
                pitch_angle_set -= (rc_data_pt->rc.ch1) / 6.0f;///< 原来*12
                /* bug,还未测量机械角 */
				Pitch_Angle_Limit(&pitch_angle_set, pitch_up_angle_limit, pitch_down_angle_limit);///<pitch角度限幅

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
        else if (robot_mode_data_pt->mode.control_device == mouse_keyboard_device_ENUM)///<键鼠模式
        {
            switch (robot_mode_data_pt->mode.mouse_keyboard_gimbal_mode)
            {
            case 1:///<手动模式（云台控制数据来自鼠标移动）
                {
                    /*目标角度设定*/
                    yaw_angle_set -= robot_mode_data_pt->virtual_rocker.ch0 / 58.0f;///<倍率需要调整。这个数据是步兵的
                    pitch_angle_set -= robot_mode_data_pt->virtual_rocker.ch1 / 1.6f;

                    //yaw角度回环
					if(yaw_angle_set>360)
						{ yaw_angle_set -= 360;}
					if(yaw_angle_set<0) 
						{yaw_angle_set += 360;}
                    
                    /*pitch轴角度限幅*/
                    Pitch_Angle_Limit(&pitch_angle_set,pitch_down_angle_limit,pitch_up_angle_limit);
                    /*两轴串级PID的角度环计算*/
                    // pitch_target_speed  = Calc_Pitch_Angle8191_Pid(pitch_angle_set,&gimbal_motor_parsed_feedback_data[pitch_motor_index]);
                    pid_out[Pitch_target_Speed] = Calc_Pitch_Angle8191_Pid(pitch_angle_set,&gimbal_motor_parsed_feedback_data[pitch_motor_index]);

                    // yaw_target_speed    = Calc_Yaw_Angle360_Pid(yaw_angle_set,imu_date_pt->pit);
                    pid_out[Yaw_target_Speed] = Calc_Yaw_Angle360_Pid(yaw_angle_set,imu_date_pt->pit);
                }

                break;
                
            case 2:///<自稳云台模式
            {
                /**目前的打算是在pitch轴加九轴陀螺仪，角度环实际角度值为陀螺仪反馈的pitch数据，设定值是
                 * 一个特殊值（只能被控制数据改变（如ch0））,ch0不变他便维持上一次值不变，这样就达到了自稳云台的效果，pitch角不被底盘晃干扰
                 * 自稳模式个人感觉非常必要
                 
                 */
                break;
            }
            case 3 :///<特殊模式（以云台坐标系为整车运动坐标系，前后左右的运动均以云台视角为准，在Chassis_task.c有具体的说明）
            {
                pitch_angle_set += robot_mode_data_pt->virtual_rocker.ch1 /3.0f;
                /*没有yaw轴角度设置*/
                /*pitch角度限制*/
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
       ///<云台串环速度环计算及发送底盘电机速度

    //    YAW_SPEED_OUTPUT_LIMIT(&yaw_target_speed,YAW_LIMIT_SPEED);  //yaw轴速度限制(移植前)
        YAW_SPEED_OUTPUT_LIMIT(&pid_out[Yaw_target_Speed],YAW_LIMIT_SPEED);  //yaw轴速度限制
       /* 之前的速度设置函数 */
        // Set_Gimbal_Motors_Speed(yaw_target_speed,
        //                         pitch_target_speed,
        //                         &gimbal_motor_parsed_feedback_data[yaw_motor_index],
        //                         &gimbal_motor_parsed_feedback_data[pitch_motor_index]);
        Set_Gimbal_Motors_Speed(pid_out[Yaw_target_Speed],
                                pid_out[Pitch_target_Speed],
                                &gimbal_motor_parsed_feedback_data[yaw_motor_index],
                                &gimbal_motor_parsed_feedback_data[pitch_motor_index]);
                            
/* 直接输出会有参数类型错误，所以使用新变量存储电机返回值 */
        // float new_moter_date = 0;
        // new_moter_date =  gimbal_motor_parsed_feedback_data[pitch_motor_index].mechanical_angle;
        // Console.print("%0.2f,%0.2f\r\n", new_moter_date,pid_out[Pitch_target_Speed]);
        osDelay(10);
    }
}

/**
 * @brief               云台相关的所有初始化
 *                      CAN2初始化，控制数据接收初始化，陀螺仪数据接收初始化
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
    imu_date_pt = Get_Imu_Date_Now();                           ///<获取陀螺仪角度
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
 * @return {Motor_Measure_t*}   解析后的云台电机数据（数组首地址）
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
