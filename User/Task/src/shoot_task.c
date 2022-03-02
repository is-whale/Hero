/**
 * @file shoot_task.c
 * @brief 发射机构，两个波轮 3508 电机的 id 分别为 201 和 202, 挂载在 can2 总线上
 *        解析后的数据分别放在 shoot_motor_feedback_data[0]、shoot_motor_feedback_data[1]
 * @version 0.1
 * @date 2021-10-02
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "shoot_task.h"

extern osThreadId waveWheelTaskHandle;

static Rc_Ctrl_t *rc_data_pt;
static Robot_control_data_t *robot_control_data_pt;

static float fric_speed = 0;
#define FRICTION_SPEED_1 fric_speed

static CAN_RxHeaderTypeDef *can2_rx_header_p;
static uint8_t *can2_rxd_data_buffer;
static const uint8_t friction_motor_num = 2;
static Motor_Measure_t friction_motor_feedback_data[friction_motor_num];
static Motor_Measure_t *wave_motor_feedback_data;
static int *erroe_integral;
static uint16_t *last_machine_angle;
static uint16_t *this_machine_angle;

static int8_t is_ok_fire = 0;

static int16_t last_wave_ch_value = 1;
static int16_t this_wave_ch_value = 1;

static const int32_t fire_one_bullet = 0x00000001;
static const int32_t fire_five_bullet = 0x00000101;
static const uint32_t wave_once_machine_angle = 50000;
const static Judge_data_t * judge_data;

void StartShootTask(void const *argument)
{
    rc_data_pt = Get_Rc_Parsed_RemoteData_Pointer();
    robot_control_data_pt = Get_Parsed_RobotMode_Pointer();
    can2_rxd_data_buffer = Get_CAN2_Rxd_Buffer();
    can2_rx_header_p = Get_CAN2_Rx_Header();
    wave_motor_feedback_data = Get_Wave_Motor_Paresed_Data();

    erroe_integral = Get_Error_Integral();
    last_machine_angle = Get_Last_Machine_Angle();
    this_machine_angle = Get_This_Machine_Angle();
	judge_data = Get_Referee_Data();
    
/* 调试区域 */
    (void)rc_data_pt; ///< 避免 warning
    (void)judge_data;
    (void)last_machine_angle;
    (void)this_machine_angle;
    (void)wave_motor_feedback_data;
    (void)erroe_integral;
    (void)wave_once_machine_angle;
/* 调试区域结束 */

    osDelay(100);

    for (;;)
    {
        Parse_Friction_Wave_Motor_Feedback_Data(can2_rx_header_p, can2_rxd_data_buffer);
        switch (robot_control_data_pt->mode.fric_cover_mode)
        {
        case fric_cover_off_mode_ENUM: ///< 0

        {
            fric_speed = 0;
            is_ok_fire = 0;

            break;
        }

        case fric_adaptive_speed_mode_ENUM: ///< 1

        {
            fric_speed = 10000;
            is_ok_fire = 1;
            break;
        }

        case fric_high_speed_mode_ENUM: ///< 2

        {
            fric_speed = 30000;
            is_ok_fire = 1;
            break;
        }

        case cover_on_ENUM: ///< 3

        {
            fric_speed = 8000;
            is_ok_fire = 1;

            break;
        }

        default:
        {
            break;
        }
        }
        if (Updata_Wave_Ch_Value(&last_wave_ch_value, &this_wave_ch_value))
        {
            osSignalSet(waveWheelTaskHandle, fire_one_bullet);
        }

        Set_Friction_Motor_Speed(-fric_speed, fric_speed, friction_motor_feedback_data);
        osDelay(10);
    }
}

/**
 * @brief                           解析云台两个摩擦轮电机和波轮电机的数据
 * @param CAN_RxHeaderTypeDef       can2 接受结构体头指针
 * @param uint8_t_*                 can2 接受到的数据
 * @retval                          void
 */
void Parse_Friction_Wave_Motor_Feedback_Data(CAN_RxHeaderTypeDef *can_rx_header, uint8_t *can2_rxd_data_buffer)
{
    static uint8_t i = 0;
    switch (can_rx_header->StdId)
    {
    case CAN_3508_FRIC_ID_LEFT:
    case CAN_3508_FRIC_ID_RIGHT:
    {
        i = can_rx_header->StdId - CAN_SHOOTER_ALL_ID - 1;
        Calculate_Motor_Data(&friction_motor_feedback_data[i], can2_rxd_data_buffer);
        break;
    }

    default:
        break;
    }
}

int8_t Updata_Wave_Ch_Value(int16_t *last_wave_ch_value, int16_t *this_wave_ch_value)
{
    *last_wave_ch_value = *this_wave_ch_value; 
    *this_wave_ch_value = rc_data_pt->rc.ch4;

    if ((*this_wave_ch_value == 660) && (*last_wave_ch_value != 660))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/* void Emission_Once_Time(void)
{
    uint32_t start_time = 0;
    uint32_t cal_pid_time = 0;
    uint32_t end_time = 0;
    wave_motor_speed = Calc_Wave_Motor_Angle8191_Pid(wave_once_machine_angle, *erroe_integral);

    ///< 初步测试，发射一颗弹丸的 pid 计算时间最多是 320 ms,时间在 290-320 之间徘徊
    while ((fabs(wave_motor_speed) > 10) && (*erroe_integral < 119900))
    {
        start_time = osKernelSysTick();
        wave_motor_speed = Calc_Wave_Motor_Angle8191_Pid(wave_once_machine_angle, *erroe_integral);
        Set_Wave_Motor_Speed(wave_motor_speed, wave_motor_feedback_data);
        osDelay(5);
        if (abs((int)(wave_motor_feedback_data->speed_rpm)) < 5)
        {
            *erroe_integral = 0;
            wave_motor_speed = 0.0;
            break;
        }
        end_time = osKernelSysTick();
        cal_pid_time += (end_time - start_time);
        if (cal_pid_time > 340)
        {
            *erroe_integral = 0;
            wave_motor_speed = 0.0;
            break;
        }
    }

    debug_print("ok time %d\r\n", cal_pid_time);
    *erroe_integral = 0;
    wave_motor_speed = 0.0;
} */

int8_t *Get_Is_OK_Fire(void)
{
    return &is_ok_fire;
}

const int32_t *Get_Fire_One_Bullet_Signal(void)
{
    return &fire_one_bullet;
}

const int32_t *Get_Fire_Five_Bullet_Signal(void)
{
    return &fire_five_bullet;
}

