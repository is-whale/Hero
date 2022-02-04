/**
 * @file chassis_task.c
 * @brief 底盘任务
 * @version 0.1
 * @date 2021-9-28
 * @copyright Copyright (c) 2021
 */
#include "chassis_task.h"

#define CHASSIS_SPEED_ZERO  0                                           ///<关闭底盘速度 1为开启
#define OUTPUT_LIMIT(data, limit) Float_Constraion(data, -limit, limit) ///<输出限幅
#define CHASSIS_MOTOR_DEFAULT_BASE_RATE 5.5f                            //底盘默认速度的倍率
#define CHASSIS_MOTOR_GYRO_BASE_RATE 5.0f 

#define POWER_LIMIT_FORM_SYSTEM 50.0f///<这里后面要换成裁判系统解析的功率限制，为了避免警告暂时使用数字代替

static Pid_Position_t chassis_follow_pid = NEW_POSITION_PID(0.26, 0, 0.8, 5000, 500, 0, 1000, 500);         ///< 底盘跟随PID
static float chassis_motor_boost_rate = 1.0f;                                                               ///<调用相应函数更改（底盘速度倍率）
static const float motor_speed_multiple = 13.5;

static Rc_Ctrl_t *rc_data_pt;                               ///< 指向解析后的遥控器结构体指针
static Robot_control_data_t *robot_mode_data_pt;            ///< 指向解析后的机器人模式结构体指针(也包括了虚拟键鼠通道的值)
static Motor_Measure_t *chassis_motor_feedback_parsed_data; ///< 解析后的底盘电机数据
static Motor_Measure_t *gimbal_motor_feedback_parsed_data;  ///< 解析后的云台电机数据
static const uint8_t *yaw_motor_index;                      ///< yaw 轴电机在云台电机数据中的下标
static const uint8_t *pitch_motor_index;                    ///< pitch 轴电机在云台电机数据中的下标
/*函数声明*/
void Chassis_Init(void);                                                ///<底盘初始化函数声明
static uint16_t Calc_Gyro_Speed_By_Power_Limit(uint16_t power_limit);   ///<计算功率限制下的小陀螺或者底盘跟随时的电机速度目标值
void Calc_Gyro_Motors_Speed(float *motors_speed, float rotate_speed,float move_direction, float x_move_speed, float y_move_speed);

void StartChassisTask(void const *argument)
{
    static float chassis_motor_speed[4] = {0.0, 0.0, 0.0, 0.0};                   ///<用于暂时存储电机速度
    float follow_pid_output;///<跟随    PID输出
    
    Chassis_Init();///<底盘初始化

    osDelay(1000); 

for (;;)
{
        ///<选择操作设备    键鼠&遥控器
        /*键鼠模式*/
    if (robot_mode_data_pt->mode.control_device == mouse_keyboard_device_ENUM) 
    {
            /**
             * @brief 选择底盘云台模式    1底盘跟随   2小陀螺   3特殊模式
             *特殊模式即不使用陀螺仪数据，以当前云台坐标系作为底盘坐标系，即前进后退以云台视角为准
             *是用于陀螺仪挂掉时备用的模式，不至于陀螺仪没了整辆车跟着完蛋
            */
        switch(robot_mode_data_pt->mode.mouse_keyboard_chassis_mode)
			{
					case mk_chassis_follow_mode_ENUM:///<底盘跟随
                {
                    //后面看需要封装成函数

                    follow_pid_output = Calc_Chassis_Follow();
              		chassis_motor_speed[0] = robot_mode_data_pt->virtual_rocker.ch2 + robot_mode_data_pt->virtual_rocker.ch3 + follow_pid_output + rc_data_pt->mouse.x/0.38f;
					chassis_motor_speed[1] = robot_mode_data_pt->virtual_rocker.ch2 - robot_mode_data_pt->virtual_rocker.ch3 + follow_pid_output + rc_data_pt->mouse.x/0.38f;
					chassis_motor_speed[2] = -robot_mode_data_pt->virtual_rocker.ch2 + robot_mode_data_pt->virtual_rocker.ch3 + follow_pid_output + rc_data_pt->mouse.x/0.38f;
					chassis_motor_speed[3] = -robot_mode_data_pt->virtual_rocker.ch2 - robot_mode_data_pt->virtual_rocker.ch3 + follow_pid_output + rc_data_pt->mouse.x/0.38f;
                    ///<乘一个倍率
                    chassis_motor_speed[0] *= (float)(CHASSIS_MOTOR_DEFAULT_BASE_RATE * chassis_motor_boost_rate);
					chassis_motor_speed[1] *= (float)(CHASSIS_MOTOR_DEFAULT_BASE_RATE * chassis_motor_boost_rate);
					chassis_motor_speed[2] *= (float)(CHASSIS_MOTOR_DEFAULT_BASE_RATE * chassis_motor_boost_rate);
				    chassis_motor_speed[3] *= (float)(CHASSIS_MOTOR_DEFAULT_BASE_RATE * chassis_motor_boost_rate);
                }

					case mk_chassis_gyro_mode_ENUM:///<底盘小陀螺

                {
                    ///<小陀螺时的电机速度计算
                    	Calc_Gyro_Motors_Speed(chassis_motor_speed, \
					Calc_Gyro_Speed_By_Power_Limit(POWER_LIMIT_FORM_SYSTEM), \
					 GM6020_YAW_Angle_To_360(gimbal_motor_feedback_parsed_data[*yaw_motor_index].mechanical_angle),\
					(float)robot_mode_data_pt->virtual_rocker.ch3 * CHASSIS_MOTOR_GYRO_BASE_RATE * chassis_motor_boost_rate, \
					(float)robot_mode_data_pt->virtual_rocker.ch2 * CHASSIS_MOTOR_GYRO_BASE_RATE * chassis_motor_boost_rate );
                }
                
			       case mk_chassis_special_mode_ENUM:///<特殊模式
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
			

    else if (robot_mode_data_pt-> mode.control_device == remote_controller_device_ENUM)///<遥控器控制
        {

        switch (robot_mode_data_pt->mode.rc_motion_mode)///<选择底盘云台的工作模式
        {

                case rc_chassis_follow_mode_ENUM: ///< 1；底盘跟随+手动瞄准
            {
                follow_pid_output = Calc_Chassis_Follow(); ///< 底盘跟随 pid,计算出 yaw 轴当前角度和 yaw 轴头之间的角度差，当作速度值加入各个轮子中

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

            case rc_chassis_gyro_mode_ENUM: ///< 2；底盘小陀螺+手动瞄准
            {
                Calc_Gyro_Motors_Speed(chassis_motor_speed,
                                       0,
                                       GM6020_YAW_Angle_To_360(gimbal_motor_feedback_parsed_data[*yaw_motor_index].mechanical_angle),
                                       rc_data_pt->rc.ch3 * 8.0f,
                                       rc_data_pt->rc.ch2 * 8.0f);
                break;
            }

            case rc_special_mode_ENUM: ///< 3；特殊模式

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
        ///< 设置底盘电机速度
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
    rc_data_pt = Get_Rc_Parsed_RemoteData_Pointer();                              // 获取解析后的遥控器数据
    robot_mode_data_pt = Get_Parsed_RobotMode_Pointer();                          //机器人模式结构体指针
    chassis_motor_feedback_parsed_data = Get_Can1_Feedback_Data();                //CAN1 总线上电机的反馈数据
    gimbal_motor_feedback_parsed_data = Get_Gimbal_Parsed_FeedBack_Data();        //CAN2总线上电机的反馈数据
    yaw_motor_index = Get_Yaw_Motor_Index();                                      // 获取 yaw 轴电机在数组中的下标
    pitch_motor_index = Get_Pitch_Motor_Index();                                  //获取pitch轴电机在数据中的下标
    (void)pitch_motor_index;
}

/**
  * @brief          计算小陀螺时，各个电机速度
  * @details        效果：底盘以设置速度小陀螺旋转或处于特殊状态，同时可以以云台枪管为头，前进后退、左右平移
  * @param[in]      {float *motors_speed }      储存电机速度的指针
  * @param[in]      {float rotate_speed }       旋转的电机速度
  * @param[in]      {float move_direction }     运动的方向角度0°~360°，以机器朝向为0°(云台GM6020电机相对底盘的角度)
  * @param[in]      {float x_move_speed}        x轴速度（X摇杆）
  * @param[in]      {float y_move_speed}        y轴速度（Y摇杆）
  * @retval                                     void
  */
void Calc_Gyro_Motors_Speed(float *motors_speed, float rotate_speed, float move_direction, float x_move_speed, float y_move_speed)
{
    ///< 判断角度是否错误
    if (move_direction > 360)
    {
        return;
    }

    ///< 角度换算弧度
    float move_radin = move_direction * 3.14159f / 180.0f;
    float radin_sin = sinf(move_radin); ///< sin(x)
    float radin_cos = cosf(move_radin);

    ///< 陀螺速度赋值
    motors_speed[0] = rotate_speed;
    motors_speed[1] = rotate_speed;
    motors_speed[2] = rotate_speed;
    motors_speed[3] = rotate_speed;

    ///< 计算速度增加值
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
 * @brief           通过读取裁判系统的限制功率计算小陀螺时各个电机（M3508）的速度（小陀螺速度目标值）
 * @details         返回的是不同实时功率下的小陀螺底盘电机速度目标值
 * @note            后面与官方开源功率控制比较之后修改一下；还有就是功率计算要改成英雄的
 * @param [in]      通过裁判系统读取的功率
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
 * @brief           计算底盘跟随 pid
 * @param[in]       void
 * @return          float 计算出的速度值
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
