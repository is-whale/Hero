#ifndef __MONITOR_TASK_H_
#define __MONITOR_TASK_H_

#include "cmsis_os.h"
#include "monitor.h"
#include "led.h"
#include "usart3.h"
#include "buzzer.h"
#include "can1_device.h"

enum moduleList
{
	remote_control = 0u, //ң����
	chassis_motor,       //���̵��
	gimbal_motor,        //��̨���
	shooter_motor,       //����������
	auto_aim,            //����NUC
	judge_system,        //����ϵͳ
	super_capacitor,     //��������
	gyroscope            //������
};

uint8_t Get_Module_Online_State(uint8_t id);
void Module_Reload(uint8_t index);
void Classis_Reload(int8_t motor_index);
void Gimbal_Reload(int8_t motor_index);

#endif // __MONITOR_TASK_H_
