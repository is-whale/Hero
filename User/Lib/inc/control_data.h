#ifndef __CONTROL_DATA_H_
#define __CONTROL_DATA_H_

#include "stm32f4xx.h"
#include "remote.h"

enum controlDevice
{
	mouse_keyboard_device_ENUM = 1u, ///< �����豸Ϊ����
	remote_controller_device_ENUM    ///< �����豸Ϊң����
};

enum rcMotionMode
{
	rc_chassis_follow_mode_ENUM = 1u,		  ///< ���̸���+�ֶ���׼
	rc_chassis_gyro_mode_ENUM,				  ///< ����С����+�ֶ���׼
	rc_autoaim_chassis_follow_mode_ENUM = 1u, ///< ���̸���+�Զ���׼
	rc_autoaim_chassis_gyro_mode_ENUM,		  ///< ����С����+�Զ���׼
	rc_special_mode_ENUM					  ///< ����ģʽ
};

enum mouseKeyboardChassisMode
{
	mk_chassis_follow_mode_ENUM = 1u, ///< ���̸���
	mk_chassis_gyro_mode_ENUM,		  ///< ����С����
	mk_chassis_special_mode_ENUM	  ///< ��������ģʽ
};

enum mouseKeyboardAimMode
{
	mk_manual_aim_mode_ENUM = 1u, ///< �ֶ���׼
	mk_auto_aim_mode_ENUM,		  ///< �Զ���׼
	mk_special_aim_mode_ENUM	  ///< ��̨����ģʽ
};

enum fricCoverMode
{
	fric_cover_off_mode_ENUM = 0u, ///< Ħ���ֹرգ����ոǹر�
	fric_adaptive_speed_mode_ENUM, ///< Ħ��������Ӧ�ٶȣ����ոǹر�
	fric_high_speed_mode_ENUM,	   ///< Ħ��������ٶ��ٶȣ����ոǹر�
	cover_on_ENUM				   ///< Ħ���ֹرգ����ոǿ���
};

enum shooterMode
{
	on_shoot_mode_ENUM = 1u, ///< ����
	rapid_fire_max_mode_ENUM ///< ��������
};

typedef struct
{
	struct
	{
		uint8_t control_device;				 ///< �����豸 1����ģʽ(rc) 2ң����ģʽ(mouse_key)
		uint8_t rc_motion_mode;				 ///< ������̨ģʽ 1���̸��� 2С���� 3���� ���̸��� 4����С���� 5����
		uint8_t mouse_keyboard_chassis_mode; ///< 1���� 2С���� 3����
		uint8_t mouse_keyboard_gimbal_mode;	 ///< 1�ֶ� 2���� 3����
		uint8_t fric_cover_mode;			 ///< Ħ����ģʽ 0�ر� 1����Ӧ 2����� 3���ո�
		uint8_t shoot_mode;					 ///< ���ģʽ 1���� 2����������ٶȣ�
		uint8_t shoot_key;                   ///< ������� 0�ر�
	} mode;

	struct
	{
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
	}parameter;

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
	
	
}Robot_control_data_t;

void Parse_Robot_Control_Data(Rc_Ctrl_t* rc_data, Rc_Ctrl_t* last_rc_data, Robot_control_data_t* robot_control_data);

#endif // __CONTROL_DATA_H_
