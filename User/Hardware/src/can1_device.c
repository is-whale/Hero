#include "can1_device.h"
#include "monitor_task.h"

static Pid_Position_t motor_fl_speed_pid = NEW_POSITION_PID(11, 0, 4.5, 2000, 16000, 0, 1000, 500);
static Pid_Position_t motor_fr_speed_pid = NEW_POSITION_PID(11, 0, 4.5, 2000, 16000, 0, 1000, 500);
static Pid_Position_t motor_bl_speed_pid = NEW_POSITION_PID(11, 0, 4.5, 2000, 16000, 0, 1000, 500);
static Pid_Position_t motor_br_speed_pid = NEW_POSITION_PID(11, 0, 4.5, 2000, 16000, 0, 1000, 500);

void Set_ChassisMotor_Speed(float speed_fl, float speed_fr, float speed_bl, float speed_br, Motor_Measure_t *chassis_motor_feedback_data)
{
    int16_t a = Pid_Position_Calc(&motor_fl_speed_pid, speed_fl, chassis_motor_feedback_data[0].speed_rpm);
    int16_t b = Pid_Position_Calc(&motor_fr_speed_pid, speed_fr, chassis_motor_feedback_data[1].speed_rpm);
    int16_t c = Pid_Position_Calc(&motor_bl_speed_pid, speed_bl, chassis_motor_feedback_data[2].speed_rpm);
    int16_t d = Pid_Position_Calc(&motor_br_speed_pid, speed_br, chassis_motor_feedback_data[3].speed_rpm);

    Can1_Send_4Msg(
        CAN_CHASSIS_ALL_ID,
        a,
        b,
        c,
        d);
}
/**
 * @brief	解析以及通知解析，底盘电机离线检测
 * @param	CAN1帧结构体
 */
void Can1_Process(CAN_RxHeaderTypeDef *can1_rx_message)
{
    switch (can1_rx_message->StdId)
    {
    case CAN_3508_M1_ID:
    case CAN_3508_M2_ID:
    case CAN_3508_M3_ID:
    case CAN_3508_M4_ID:
    {
        Info_Can1_ParseData_Task();
        uint8_t i = 0;
        //处理电机ID号
        i = can1_rx_message->StdId - CAN_3508_M1_ID;
        //处理电机数据宏函数
        /* 底盘解析 */
        //记录时间
        Classis_Reload(i);
        break;
    }
    case SUPER_CAPACITOR_ID:
    {
        /* 解析超级电容 */
        Info_Super_Capacitor_Parse_Data();
        break;
    }
    default:
    {
        break;
    }
    }
}
