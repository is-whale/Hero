#ifndef __CONTROL_DATA_H_
#define __CONTROL_DATA_H_

#include "stm32f4xx.h"
#include "remote.h"

typedef struct
{
	struct
	{
		uint8_t control_device;				 //�����豸 1����ģʽ(rc) 2ң����ģʽ(mouse_key)
		uint8_t rc_motion_mode;				 //������̨ģʽ 1���̸��� 2С���� 3������� 4����С���� 5����
		uint8_t mouse_keyboard_chassis_mode; //1���� 2С���� 3����
		uint8_t mouse_keyboard_gimbal_mode;	 //1�ֶ� 2���� 3����
		uint8_t shoot_mode;					 //���ģʽ 1���� 2���������٣� 3����������ٶȣ�
		uint8_t fric_cover_mode;			 //Ħ����ģʽ 0�ر� 1����Ӧ 2����� 3���ո�

		//const�������ٶ�
		int16_t mouse_x_max_value;
		int16_t mouse_y_max_value;

		//const������ݱ���(������)
		float mouse_x_sensitivity;
		float mouse_y_sensitivity;

		//���̼��ٶ�
		int16_t chassis_acceleration;
		//��������ٶ�
		int16_t chassis_max_speed; 

	} mode;

	struct
	{
		int16_t ch0; //���
		int16_t ch1; //���
		int16_t ch2; //���� ����WS
		int16_t ch3; //ǰ�� ����AD
	} virtual_rocker;

	//ctrl�����л�
	//���ݿ��ء�����-1 0 1
	//�������ģʽ
	//���ߵ���һ��С���� ����

	//shift���ƣ���ס��
	//�ٶȡ����ٶȱ��� x = 0~1
	//���ٶȱ��� 1+x
	//�ٶȱ��� 1+x
	
	
}Robot_contral_data_t;

// void Parse_Robot_Contral_Data(Rc_Ctrl_t* rc_data, Rc_Ctrl_t* last_rc_data, Robot_contral_data_t* robot_contral_data);

#endif // __CONTROL_DATA_H_
