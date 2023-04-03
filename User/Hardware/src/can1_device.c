#include "can1_device.h"
#include "monitor_task.h"
/* ���빦�ʿ��ƺ��� */
#include "chassis_task.h"

static Pid_Position_t motor_fl_speed_pid = NEW_POSITION_PID(9, 0, 4.5, 2000, 16000, 0, 1000, 500);
static Pid_Position_t motor_fr_speed_pid = NEW_POSITION_PID(9, 0, 4.5, 2000, 16000, 0, 1000, 500);
static Pid_Position_t motor_bl_speed_pid = NEW_POSITION_PID(9, 0, 4.5, 2000, 16000, 0, 1000, 500);
static Pid_Position_t motor_br_speed_pid = NEW_POSITION_PID(9, 0, 4.5, 2000, 16000, 0, 1000, 500);

void Set_ChassisMotor_Speed(float speed_fl, float speed_fr, float speed_bl, float speed_br, Motor_Measure_t *chassis_motor_feedback_data)
{
    int16_t speed1 = Pid_Position_Calc(&motor_fl_speed_pid, speed_fl, chassis_motor_feedback_data[0].speed_rpm);
    int16_t speed2 = Pid_Position_Calc(&motor_fr_speed_pid, speed_fr, chassis_motor_feedback_data[1].speed_rpm);
    int16_t speed3 = Pid_Position_Calc(&motor_bl_speed_pid, speed_bl, chassis_motor_feedback_data[2].speed_rpm);
    int16_t speed4 = Pid_Position_Calc(&motor_br_speed_pid, speed_br, chassis_motor_feedback_data[3].speed_rpm);

    chassis_power_control(&speed1,&speed2,&speed3,&speed4);
    Can1_Send_4Msg(
        CAN_CHASSIS_ALL_ID,
        speed1,
        speed2,
        speed3,
        speed4);
}
/**
 * @brief	�����Լ�֪ͨ���������̵�����߼��
 * @param	CAN1֡�ṹ��
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
        Info_Can1_ParseData_Task();///< ����CAN1���ݣ�ɾ���˽�������
        uint8_t i = 0;
        //������ID��
        i = can1_rx_message->StdId - CAN_3508_M1_ID;
        //���������ݺ꺯��
        /* ���ص������߼�� */
        Classis_Reload(i);
        break;
    }
    case SUPER_CAPACITOR_ID:
    {
        /* ֪ͨ�������ݽ��� */
        Info_Super_Capacitor_Parse_Data();
        break;
    }
    default:
    {
        break;
    }
    }
}
/**
 * @brief   �������ݷ��ͺ���
 * @param
 *
  */
void Set_Super_Capacitor(uint16_t target_power)
{
     Can1_Send_4Msg(SUPER_CAPACITOR_SEND_ID, target_power, 0, 0, 0);
}
