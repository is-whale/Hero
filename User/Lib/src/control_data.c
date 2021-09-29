#include "control_data.h"

static void Robot_Rc_Mode_Change_Control(Rc_Ctrl_t* rc_data, Rc_Ctrl_t* last_rc_data, Robot_control_data_t* robot_control_data);
static void Wasd_Key_To_Virtual_Rocker(Rc_Ctrl_t* rc_data, Robot_control_data_t* robot_control_data);
static void Mouse_To_Virtual_Rocker(Rc_Ctrl_t* rc_data, Robot_control_data_t* robot_control_data);
static void Switch_Mouse_Key_Change(Rc_Ctrl_t* rc_data, Rc_Ctrl_t* last_rc_data, Robot_control_data_t* robot_control_data);
static void Shoot_Key_Control(Rc_Ctrl_t* rc_data, Rc_Ctrl_t* last_rc_data, Robot_control_data_t* robot_control_data);

void Parse_Robot_Control_Data(Rc_Ctrl_t* rc_data, Rc_Ctrl_t* last_rc_data, Robot_control_data_t* robot_control_data)
{
	Robot_Rc_Mode_Change_Control(rc_data, last_rc_data, robot_control_data); ///< ������ģʽ�任��Ӧ
	Wasd_Key_To_Virtual_Rocker(rc_data, robot_control_data); ///< WASD����Ϊҡ��
	Mouse_To_Virtual_Rocker(rc_data, robot_control_data); ///< �������Ϊҡ��
	Switch_Mouse_Key_Change(rc_data, last_rc_data, robot_control_data); ///< ��Ӧ���̿���
	Shoot_Key_Control(rc_data, last_rc_data, robot_control_data); ///< �����ť
}

static void Robot_Rc_Mode_Change_Control(Rc_Ctrl_t* rc_data, Rc_Ctrl_t* last_rc_data, Robot_control_data_t* robot_control_data)
{
#define ROBOT_MODE     robot_control_data->mode
#define S1_VALUE       rc_data->rc.s1
#define S2_VALUE       rc_data->rc.s2
#define OLD_S1_VALUE   last_rc_data->rc.s1
#define OLD_S2_VALUE   last_rc_data->rc.s2
#define S1_CHANGED_TO(a,b) ((OLD_S1_VALUE == (a)) && (S1_VALUE == (b)))
#define S2_CHANGED_TO(a,b) ((OLD_S2_VALUE == (a)) && (S2_VALUE == (b)))

	/* �ٿ��豸ѡ�� */
	if(S2_CHANGED_TO(3,2))
	{
		ROBOT_MODE.control_device++;
		if(ROBOT_MODE.control_device==3) ROBOT_MODE.control_device=1;
		// Set_Beep_Time(ROBOT_MODE.control_device, 1000, 55);
	}
	
	/* ������̨ģʽ */
	if(S2_CHANGED_TO(3,1))
	{
		ROBOT_MODE.rc_motion_mode++;
		if(ROBOT_MODE.rc_motion_mode==6) ROBOT_MODE.rc_motion_mode=1;
		// Set_Beep_Time(ROBOT_MODE.rc_motion_mode, 1200, 50);
	}

	if(ROBOT_MODE.control_device != 2)
	{
		return;
	}

	/* Ħ�����ٶ�ģʽ���ոǿ��� */
	if(S1_CHANGED_TO(3,2))
	{
		ROBOT_MODE.fric_cover_mode++;
		if(ROBOT_MODE.fric_cover_mode==4) ROBOT_MODE.fric_cover_mode=0;
		// Set_Beep_Time(ROBOT_MODE.fric_cover_mode+1, 800, 60);
	}

	/* ���ģʽѡ���� */
	if(S1_CHANGED_TO(3,1))
	{
		ROBOT_MODE.shoot_mode++;
		if(ROBOT_MODE.shoot_mode==5) ROBOT_MODE.shoot_mode=1;
		// Set_Beep_Time(ROBOT_MODE.shoot_mode, 600, 65);
	}

#undef ROBOT_MODE
#undef S1_VALUE
#undef S2_VALUE
#undef OLD_S1_VALUE
#undef OLD_S2_VALUE
#undef S1_CHANGED_TO
#undef S2_CHANGED_TO
}

static void Wasd_Key_To_Virtual_Rocker(Rc_Ctrl_t* rc_data, Robot_control_data_t* robot_control_data)
{
#define KEY_W            0x01
#define KEY_A            0x04
#define KEY_S            0x02
#define KEY_D            0x08
#define RC_KEY           rc_data->key.value
#define KEY_PRESSED(key) (RC_KEY & key)
#define VIRTUAL_ROCKER_STEP 10
#define VIRTUAL_ROCKER    robot_control_data->virtual_rocker
	/*ֻ����Wʱ��������S*/
	if(KEY_PRESSED(KEY_W) && !(KEY_PRESSED(KEY_S)))
	{
		if(VIRTUAL_ROCKER.ch3 < 0)
		{
			VIRTUAL_ROCKER.ch3 += VIRTUAL_ROCKER_STEP;
		}
		if(VIRTUAL_ROCKER.ch3 < 600)
		{
			VIRTUAL_ROCKER.ch3 += VIRTUAL_ROCKER_STEP;
		}
	}
	/*ֻ����Sʱ��������W*/
	else if(KEY_PRESSED(KEY_S) && !(KEY_PRESSED(KEY_W)))
	{
		if(VIRTUAL_ROCKER.ch3 > 0)
		{
			VIRTUAL_ROCKER.ch3 -= VIRTUAL_ROCKER_STEP;
		}
		if(VIRTUAL_ROCKER.ch3 > -600)
		{
			VIRTUAL_ROCKER.ch3 -= VIRTUAL_ROCKER_STEP;
		}
	}
	/*W��S��δ����ʱ*/
	else
	{
		if(VIRTUAL_ROCKER.ch3 < 0)
		{
			VIRTUAL_ROCKER.ch3 += VIRTUAL_ROCKER_STEP;
			if(VIRTUAL_ROCKER.ch3 > 0)
				VIRTUAL_ROCKER.ch3 = 0;
		}
		else if(VIRTUAL_ROCKER.ch3 > 0)
		{
			VIRTUAL_ROCKER.ch3 -= VIRTUAL_ROCKER_STEP;
			if(VIRTUAL_ROCKER.ch3 < 0)
				VIRTUAL_ROCKER.ch3 = 0;
		}
	}

	/*ֻ����Dʱ*/
	if(KEY_PRESSED(KEY_D) && !(KEY_PRESSED(KEY_A)))
	{
		if(VIRTUAL_ROCKER.ch2 < 0)
		{
			VIRTUAL_ROCKER.ch2 += VIRTUAL_ROCKER_STEP;
		}
		if(VIRTUAL_ROCKER.ch2 < 600)
		{
			VIRTUAL_ROCKER.ch2 += VIRTUAL_ROCKER_STEP;
		}
	}
	/*ֻ����Aʱ*/
	else if(KEY_PRESSED(KEY_A) && !(KEY_PRESSED(KEY_D)))
	{
		if(VIRTUAL_ROCKER.ch2 > 0)
		{
			VIRTUAL_ROCKER.ch2 -= VIRTUAL_ROCKER_STEP;
		}
		if(VIRTUAL_ROCKER.ch2 > -600)
		{
			VIRTUAL_ROCKER.ch2 -= VIRTUAL_ROCKER_STEP;
		}

	}
	/*A��D��δ����ʱ*/
	else
	{
		if(VIRTUAL_ROCKER.ch2 < 0)
		{
			VIRTUAL_ROCKER.ch2 += VIRTUAL_ROCKER_STEP;
			if(VIRTUAL_ROCKER.ch2 > 0)
				VIRTUAL_ROCKER.ch2 = 0;
		}
		else if(VIRTUAL_ROCKER.ch2 > 0)
		{
			VIRTUAL_ROCKER.ch2 -= VIRTUAL_ROCKER_STEP;
			if(VIRTUAL_ROCKER.ch2 < 0)
				VIRTUAL_ROCKER.ch2 = 0;
		}
	}

}

static void Mouse_To_Virtual_Rocker(Rc_Ctrl_t* rc_data, Robot_control_data_t* robot_control_data)
{
	//TODO �������������
	//TODO �����ֵ*������
}

/* ��Ӧ�����л�ģʽ */
static void Switch_Mouse_Key_Change(Rc_Ctrl_t* rc_data, Rc_Ctrl_t* last_rc_data, Robot_control_data_t* robot_control_data)
{
	//�ж��ǲ��Ǽ���ģʽ
	if(robot_control_data->mode.control_device != 1)
	{
		return;
	}

	// //��סshift���̼���
	// if( RC_KEY_PRESSED(KEY_VALUE, KEY_SHIFT) )
	// {
	// 	Change_Chassis_Motor_Boost_Rate(1, 2.7); //����
	// }
	// else
	// {
	// 	Change_Chassis_Motor_Boost_Rate(-1, 2.7); //����
	// }

	// //����ģʽ(����ctrl���ĵ���ģʽ1����ģʽ2С����ģʽ)
	// if(KEY_CLICKED(KEY_CTRL))
	// {
	// 	if(robot_mode->mouse_key_chassis_mode == 1)
	// 	{
	// 		robot_mode->mouse_key_chassis_mode = 2;
	// 		Set_Beep_Time(2, 1200, 50);
	// 	}
	// 	else
	// 	{
	// 		robot_mode->mouse_key_chassis_mode = 1;
	// 		Set_Beep_Time(1, 1200, 50);
	// 	}
	// }

	// //��̨ģʽ G (�ֶ����� ����ģʽ)
	// if(KEY_CLICKED(KEY_G))
	// {
	// 	if(robot_mode->mouse_key_gimbal_mode == 1)
	// 	{
	// 		robot_mode->mouse_key_gimbal_mode = 2;
	// 		Set_Beep_Time(4, 1200, 50);
	// 	}
	// 	else
	// 	{
	// 		robot_mode->mouse_key_gimbal_mode = 1;
	// 		Set_Beep_Time(3, 1200, 50);
	// 	}
	// }

	// //����ģʽ
	// if( KEY_CLICKED(KEY_Z) )
	// {
	// 	robot_mode->mouse_key_chassis_mode = 3;
	// 	robot_mode->mouse_key_gimbal_mode = 3;
	// 	Set_Beep_Time(5, 1200, 50);
	// }

	// //������ģʽ���
	// if(robot_mode->mouse_key_chassis_mode == 3 && robot_mode->mouse_key_gimbal_mode != 3)
	// {
	// 	robot_mode->mouse_key_chassis_mode = 1;
	// }
	// if(robot_mode->mouse_key_chassis_mode != 3 && robot_mode->mouse_key_gimbal_mode == 3)
	// {
	// 	robot_mode->mouse_key_gimbal_mode = 1;
	// }

	// //����ģʽ R �����ص��ոǣ�
	// if(KEY_CLICKED(KEY_R))
	// {
	// 	if(robot_mode->fric_cover_mode == 3)
	// 	{
	// 		robot_mode->fric_cover_mode = 0;
	// 		Set_Beep_Time(1, 800, 60);
	// 	}
	// 	else
	// 	{
	// 		robot_mode->fric_cover_mode = 3;
	// 		Set_Beep_Time(4, 800, 60);
	// 	}
	// }

	// //Ħ����ģʽ E
	// if(KEY_CLICKED(KEY_E))
	// {
	// 	if(robot_mode->fric_cover_mode == 1)
	// 	{
	// 		robot_mode->fric_cover_mode = 0;
	// 		Set_Beep_Time(1, 800, 60);
	// 	}
	// 	else
	// 	{
	// 		robot_mode->fric_cover_mode = 1;
	// 		Set_Beep_Time(2, 800, 60);
	// 	}
	// }
}

/* ��Ӧ���ģʽ */
static void Shoot_Key_Control(Rc_Ctrl_t* rc_data, Rc_Ctrl_t* last_rc_data, Robot_control_data_t* robot_control_data)
{
	//ң����ģʽ
	if(robot_control_data->mode.control_device == remote_controller_device_ENUM)
	{
		/* �����ť */
		if (robot_control_data->mode.shoot_mode == on_shoot_mode_ENUM)
		{
			if (last_rc_data->rc.ch4 <= 120 && rc_data->rc.ch4 > 120)
			{
				robot_control_data->mode.shoot_key = on_shoot_mode_ENUM;
			}
		}
		else
		{
			if (rc_data->rc.ch4 > 120)
			{
				robot_control_data->mode.shoot_key = robot_control_data->mode.shoot_mode;
			}
			else
			{
				robot_control_data->mode.shoot_key = 0;
			}
		}
	}
	
	// //����ģʽ
	// else if(robot_mode.control_device == 1)
	// {
	// 	if( Is_Id1_17mm_Excess_Heat(judge_data) == 0 )  //�ж��Ƿ�����
	// 	{
	// 		if(remote_controller.mouse.press_l)
	// 		{
	// 			Set_Shoot_key(3);
	// 		}
	// 		else
	// 		{
	// 			Set_Shoot_key(0);
	// 		}

	// 		if( remote_controller.mouse.press_r && (!last_time_rc.mouse.press_r) )
	// 		{
	// 			Set_Shoot_key(2);
	// 		}
	// 	}
	// 	else
	// 	{
	// 		Set_Shoot_key(0);
	// 	}

	// }

}


