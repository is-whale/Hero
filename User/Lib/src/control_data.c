#include "control_data.h"

static void Robot_Rc_Mode_Change_Control(Rc_Ctrl_t *rc_data, Rc_Ctrl_t *last_rc_data, Robot_control_data_t *robot_control_data);
static void Wasd_Key_To_Virtual_Rocker(Rc_Ctrl_t *rc_data, Robot_control_data_t *robot_control_data);
static void Mouse_To_Virtual_Rocker(Rc_Ctrl_t *rc_data, Robot_control_data_t *robot_control_data);
static void Switch_Mouse_Key_Change(Rc_Ctrl_t *rc_data, Rc_Ctrl_t *last_rc_data, Robot_control_data_t *robot_control_data);
static void Shoot_Key_Control(Rc_Ctrl_t *rc_data, Rc_Ctrl_t *last_rc_data, Robot_control_data_t *robot_control_data);

/**
 *@brief		���û����˳�ʼʱ��ģʽ
 @param			�������ݽṹ���ַ
 @retval		void
  */
void Control_Data_Init(Robot_control_data_t *robot_control_data)
{
	robot_control_data->mode.control_device = mouse_keyboard_device_ENUM;				///< ������
	robot_control_data->mode.rc_motion_mode = rc_special_mode_ENUM;						///< ����ģʽ��ң����ģʽ��
	robot_control_data->mode.mouse_keyboard_chassis_mode = mk_chassis_follow_mode_ENUM; ///< ���̸��� ������ģʽ��
	robot_control_data->mode.mouse_keyboard_gimbal_mode = mk_manual_aim_mode_ENUM;		///< �ֶ���׼ģʽ����̨�����˶���
	robot_control_data->mode.gyro_direction = gyro_positive_ENUM;						///< С���ݷ�������
	robot_control_data->mode.fric_cover_mode = fric_cover_off_mode_ENUM;				///< �ر�Ħ����
	robot_control_data->mode.shoot_mode = on_shoot_mode_ENUM;							///< ����ģʽ
	robot_control_data->mode.shoot_key = 0;												///< �ر��������

#define MOUSE_X_MAX_VALUE 5000	 ///< X�����޷�
#define MOUSE_Y_MAX_VALUE 5000	 ///< Y�����޷�
#define MOUSE_X_SENSITIVITY 0.8f ///< X������
#define MOUSE_Y_SENSITIVITY 0.8f

	robot_control_data->parameter.mouse_x_max_value = MOUSE_X_MAX_VALUE;
	robot_control_data->parameter.mouse_y_max_value = MOUSE_Y_MAX_VALUE;
	robot_control_data->parameter.mouse_x_sensitivity = MOUSE_X_SENSITIVITY;
	robot_control_data->parameter.mouse_y_sensitivity = MOUSE_Y_SENSITIVITY;
	robot_control_data->parameter.chassis_acceleration = 10;
	robot_control_data->parameter.chassis_max_speed = 660;

#undef MOUSE_X_MAX_VALUE
#undef MOUSE_Y_MAX_VALUE
#undef MOUSE_X_SENSITIVITY
#undef MOUSE_Y_SENSITIVITY

	robot_control_data->virtual_rocker.ch0 = 0;
	robot_control_data->virtual_rocker.ch1 = 0;
	robot_control_data->virtual_rocker.ch2 = 0;
	robot_control_data->virtual_rocker.ch3 = 0;
}
/**
 * @brief ���������˿���ģʽ
 * @param[in] {rc_data}   					���ν�����ң��������
 * @param[in] {last_rc_data}   				�ϴν�����ң��������
 * @param[in] {uint16_t}					������ģʽ��������
 * @retval    {uint8_t}						1 : ���ݽ�����У��ɹ�
 * 											0 : ���ݽ���У��ʧ��
 */
void Parse_Robot_Control_Data(Rc_Ctrl_t *rc_data, Rc_Ctrl_t *last_rc_data, Robot_control_data_t *robot_control_data)
{
	Robot_Rc_Mode_Change_Control(rc_data, last_rc_data, robot_control_data); ///< ������ģʽ�任��Ӧ
	Wasd_Key_To_Virtual_Rocker(rc_data, robot_control_data);				 ///< WASD����Ϊҡ��
	Mouse_To_Virtual_Rocker(rc_data, robot_control_data);					 ///< �������Ϊҡ��
	Switch_Mouse_Key_Change(rc_data, last_rc_data, robot_control_data);		 ///< ��Ӧ���̿���
	Shoot_Key_Control(rc_data, last_rc_data, robot_control_data);			 ///< �����ť
}

static void Robot_Rc_Mode_Change_Control(Rc_Ctrl_t *rc_data, Rc_Ctrl_t *last_rc_data, Robot_control_data_t *robot_control_data)
{
#define ROBOT_MODE robot_control_data->mode
#define S1_VALUE rc_data->rc.s1
#define S2_VALUE rc_data->rc.s2
#define OLD_S1_VALUE last_rc_data->rc.s1
#define OLD_S2_VALUE last_rc_data->rc.s2
#define S1_CHANGED_TO(a, b) ((OLD_S1_VALUE == (a)) && (S1_VALUE == (b)))
#define S2_CHANGED_TO(a, b) ((OLD_S2_VALUE == (a)) && (S2_VALUE == (b)))

	/* �ٿ��豸ѡ�� */
	if (S2_CHANGED_TO(3, 2))
	{
		ROBOT_MODE.control_device++;
		if (ROBOT_MODE.control_device == 3)
			ROBOT_MODE.control_device = 1;
		Set_Beep_Time(ROBOT_MODE.control_device, 1000, 55);
	}

	/* ������̨ģʽ */
	if (S2_CHANGED_TO(3, 1))
	{
		ROBOT_MODE.rc_motion_mode++;
		if (ROBOT_MODE.rc_motion_mode == 6)
			ROBOT_MODE.rc_motion_mode = 1;
		Set_Beep_Time(ROBOT_MODE.rc_motion_mode, 1200, 50);
	}

	if (ROBOT_MODE.control_device != 2)
	{
		return;
	}

	/* Ħ�����ٶ�ģʽ���ոǿ��� */
	if (S1_CHANGED_TO(3, 2))
	{
		ROBOT_MODE.fric_cover_mode++;
		if (ROBOT_MODE.fric_cover_mode == 4)
			ROBOT_MODE.fric_cover_mode = 0;
		Set_Beep_Time(ROBOT_MODE.fric_cover_mode + 1, 800, 60);
	}

	/* ���ģʽѡ���� */
	if (S1_CHANGED_TO(3, 1))
	{
		ROBOT_MODE.shoot_mode++;
		if (ROBOT_MODE.shoot_mode == 3)
			ROBOT_MODE.shoot_mode = 1;
		Set_Beep_Time(ROBOT_MODE.shoot_mode, 600, 65);
	}

#undef ROBOT_MODE
#undef S1_VALUE
#undef S2_VALUE
#undef OLD_S1_VALUE
#undef OLD_S2_VALUE
#undef S1_CHANGED_TO
#undef S2_CHANGED_TO
}
/**
 * @brief               					�������ݽ���Ϊ����ҡ��
 * @param[in] {Rc_Ctrl_t*}   				ң�������ݻ�����
 * @param[in] {Robot_control_data_t*}   	ң��������ָ��ṹ��
 * @retval    {uint8_t}						��
 */
static void Wasd_Key_To_Virtual_Rocker(Rc_Ctrl_t *rc_data, Robot_control_data_t *robot_control_data)
{
///<��֤����ֲ��
#define KEY_W 0x01
#define KEY_A 0x04
#define KEY_S 0x02
#define KEY_D 0x08
#define RC_KEY rc_data->key.value
#define KEY_PRESSED(key) (RC_KEY & key)
#define VIRTUAL_ROCKER_STEP robot_control_data->parameter.chassis_acceleration ///<���̼��ٶ�
#define VIRTUAL_ROCKER_MIN (-robot_control_data->parameter.chassis_max_speed)  ///<���̷�������ٶ�
#define VIRTUAL_ROCKER_MAX robot_control_data->parameter.chassis_max_speed	   ///<������������ٶ�
#define VIRTUAL_ROCKER robot_control_data->virtual_rocker

	/* ch3 min max���� */
	if (VIRTUAL_ROCKER.ch3 > VIRTUAL_ROCKER_MAX)
	{
		Int16_Minus_Limit(&VIRTUAL_ROCKER.ch3, VIRTUAL_ROCKER_STEP, VIRTUAL_ROCKER_MAX);
	}
	else if (VIRTUAL_ROCKER.ch3 < VIRTUAL_ROCKER_MIN)
	{
		Int16_Add_Limit(&VIRTUAL_ROCKER.ch3, VIRTUAL_ROCKER_STEP, VIRTUAL_ROCKER_MIN);
	}

	/*ֻ����Wʱ��������S*/
	if (KEY_PRESSED(KEY_W) && !(KEY_PRESSED(KEY_S)))
	{
		if (VIRTUAL_ROCKER.ch3 < 0)
		{
			VIRTUAL_ROCKER.ch3 += VIRTUAL_ROCKER_STEP;
		}
		if (VIRTUAL_ROCKER.ch3 < VIRTUAL_ROCKER_MAX)
		{
			Int16_Add_Limit(&VIRTUAL_ROCKER.ch3, VIRTUAL_ROCKER_STEP, VIRTUAL_ROCKER_MAX);
		}
	}
	/*ֻ����Sʱ��������W*/
	else if (KEY_PRESSED(KEY_S) && !(KEY_PRESSED(KEY_W)))
	{
		if (VIRTUAL_ROCKER.ch3 > 0)
		{
			VIRTUAL_ROCKER.ch3 -= VIRTUAL_ROCKER_STEP;
		}
		if (VIRTUAL_ROCKER.ch3 > VIRTUAL_ROCKER_MIN)
		{
			Int16_Minus_Limit(&VIRTUAL_ROCKER.ch3, VIRTUAL_ROCKER_STEP, VIRTUAL_ROCKER_MIN);
		}
	}
	/*W��S��δ����ʱ*/
	else
	{
		if (VIRTUAL_ROCKER.ch3 < 0)
		{
			Int16_Add_Limit(&VIRTUAL_ROCKER.ch3, VIRTUAL_ROCKER_STEP, 0);
		}
		else if (VIRTUAL_ROCKER.ch3 > 0)
		{
			Int16_Minus_Limit(&VIRTUAL_ROCKER.ch3, VIRTUAL_ROCKER_STEP, 0);
		}
	}

	/* ch2 min max���� */
	if (VIRTUAL_ROCKER.ch2 > VIRTUAL_ROCKER_MAX)
	{
		Int16_Minus_Limit(&VIRTUAL_ROCKER.ch2, VIRTUAL_ROCKER_STEP, VIRTUAL_ROCKER_MAX);
	}
	else if (VIRTUAL_ROCKER.ch2 < VIRTUAL_ROCKER_MIN)
	{
		Int16_Add_Limit(&VIRTUAL_ROCKER.ch2, VIRTUAL_ROCKER_STEP, VIRTUAL_ROCKER_MIN);
	}

	/*ֻ����Dʱ*/
	if (KEY_PRESSED(KEY_D) && !(KEY_PRESSED(KEY_A)))
	{
		if (VIRTUAL_ROCKER.ch2 < 0)
		{
			VIRTUAL_ROCKER.ch2 += VIRTUAL_ROCKER_STEP;
		}
		if (VIRTUAL_ROCKER.ch2 < VIRTUAL_ROCKER_MAX)
		{
			Int16_Add_Limit(&VIRTUAL_ROCKER.ch2, VIRTUAL_ROCKER_STEP, VIRTUAL_ROCKER_MAX);
		}
	}
	/*ֻ����Aʱ*/
	else if (KEY_PRESSED(KEY_A) && !(KEY_PRESSED(KEY_D)))
	{
		if (VIRTUAL_ROCKER.ch2 > 0)
		{
			VIRTUAL_ROCKER.ch2 -= VIRTUAL_ROCKER_STEP;
		}
		if (VIRTUAL_ROCKER.ch2 > VIRTUAL_ROCKER_MIN)
		{
			Int16_Minus_Limit(&VIRTUAL_ROCKER.ch2, VIRTUAL_ROCKER_STEP, VIRTUAL_ROCKER_MIN);
		}
	}
	/*A��D��δ����ʱ*/
	else
	{
		if (VIRTUAL_ROCKER.ch2 < 0)
		{
			Int16_Add_Limit(&VIRTUAL_ROCKER.ch2, VIRTUAL_ROCKER_STEP, 0);
		}
		else if (VIRTUAL_ROCKER.ch2 > 0)
		{
			Int16_Minus_Limit(&VIRTUAL_ROCKER.ch2, VIRTUAL_ROCKER_STEP, 0);
		}
	}

#undef KEY_W
#undef KEY_A
#undef KEY_S
#undef KEY_D
#undef RC_KEY
#undef KEY_PRESSED
#undef VIRTUAL_ROCKER_STEP
#undef VIRTUAL_ROCKER_MIN
#undef VIRTUAL_ROCKER_MAX
#undef VIRTUAL_ROCKER
	/*�൱��W,S,A,D,ֻ����Ҫ���õ��м�ֵ������������������ҡ�����ݣ���Ϊ����ʹ�õ�����*/
}
/** @brief								����ƶ�����Ϊ����ҡ��ͨ��ֵ
 *  @param [in] Rc_Ctrl_t				���յ�ң�������ݽṹ��ָ��
 *  @param [in] Robot_control_data_t	�����˿������ݽṹ��ָ��
 *
 */
static void Mouse_To_Virtual_Rocker(Rc_Ctrl_t *rc_data, Robot_control_data_t *robot_control_data)
{
	//�������������
	robot_control_data->virtual_rocker.ch0 = Int16_Limit(rc_data->mouse.x, -robot_control_data->parameter.mouse_x_max_value, robot_control_data->parameter.mouse_x_max_value);
	robot_control_data->virtual_rocker.ch1 = Int16_Limit(rc_data->mouse.y, -robot_control_data->parameter.mouse_y_max_value, robot_control_data->parameter.mouse_y_max_value);

	//�����ֵ*������
	robot_control_data->virtual_rocker.ch0 *= robot_control_data->parameter.mouse_x_sensitivity;
	robot_control_data->virtual_rocker.ch1 *= robot_control_data->parameter.mouse_y_sensitivity;
}

/* ��Ӧ�����л�ģʽ */
static void Switch_Mouse_Key_Change(Rc_Ctrl_t *rc_data, Rc_Ctrl_t *last_rc_data, Robot_control_data_t *robot_control_data)
{
#define KEY_CTRL 0x20
#define KEY_SHIFT 0x10
#define KEY_Z 0x800
#define KEY_X 0x1000
#define KEY_C 0x2000
#define KEY_V 0x4000
#define KEY_B 0x8000
#define KEY_F 0x200
#define KEY_G 0x400
#define KEY_Q 0x40
#define KEY_E 0x80
#define KEY_R 0x100
#define KEY_VALUE rc_data->key.value
#define OLD_KEY_VALUE last_rc_data->key.value
#define KEY_PRESSED(key) (KEY_VALUE & key)
#define KEY_CLICKED(key) ((KEY_VALUE & key) && (!(OLD_KEY_VALUE & key)))

	//�ж��ǲ��Ǽ���ģʽ
	if (robot_control_data->mode.control_device != 1)
	{
		return;
	}

	//��סshift���̼���
	if (KEY_PRESSED(KEY_SHIFT))
	{
		robot_control_data->parameter.chassis_acceleration = 50;
		robot_control_data->parameter.chassis_max_speed = 660;
	}
	else
	{
		robot_control_data->parameter.chassis_acceleration = 16;
		robot_control_data->parameter.chassis_max_speed = 360;
	}

	//����ģʽ(����ctrl���ĵ���ģʽ1����ģʽ2С����ģʽ)
	if (KEY_CLICKED(KEY_CTRL))
	{
		if (robot_control_data->mode.mouse_keyboard_chassis_mode == mk_chassis_follow_mode_ENUM)
		{
			robot_control_data->mode.mouse_keyboard_chassis_mode = mk_chassis_gyro_mode_ENUM;
			Set_Beep_Time(2, 1200, 50);
		}
		else
		{
			robot_control_data->mode.mouse_keyboard_chassis_mode = mk_chassis_follow_mode_ENUM;
			Set_Beep_Time(1, 1200, 50);
		}
	}

	//��̨ģʽ G (�ֶ����� ����ģʽ)
	if (KEY_CLICKED(KEY_G))
	{
		if (robot_control_data->mode.mouse_keyboard_gimbal_mode == mk_manual_aim_mode_ENUM)
		{
			robot_control_data->mode.mouse_keyboard_gimbal_mode = mk_auto_aim_mode_ENUM;
			Set_Beep_Time(4, 1200, 50);
		}
		else
		{
			robot_control_data->mode.mouse_keyboard_gimbal_mode = mk_manual_aim_mode_ENUM;
			Set_Beep_Time(3, 1200, 50);
		}
	}

	//����ģʽ
	if (KEY_CLICKED(KEY_V))
	{
		robot_control_data->mode.mouse_keyboard_chassis_mode = mk_chassis_special_mode_ENUM;
		robot_control_data->mode.mouse_keyboard_gimbal_mode = mk_special_aim_mode_ENUM;
		Set_Beep_Time(5, 1200, 50);
	}

	//������ģʽ���
	if (robot_control_data->mode.mouse_keyboard_chassis_mode == 3 && robot_control_data->mode.mouse_keyboard_gimbal_mode != 3)
	{
		robot_control_data->mode.mouse_keyboard_chassis_mode = 1;
	}
	if (robot_control_data->mode.mouse_keyboard_chassis_mode != 3 && robot_control_data->mode.mouse_keyboard_gimbal_mode == 3)
	{
		robot_control_data->mode.mouse_keyboard_gimbal_mode = 1;
	}

	//����ģʽ R �����ص��ոǣ�
	if (KEY_CLICKED(KEY_R))
	{
		if (robot_control_data->mode.fric_cover_mode == cover_on_ENUM)
		{
			robot_control_data->mode.fric_cover_mode = fric_cover_off_mode_ENUM;
			Set_Beep_Time(1, 800, 60);
		}
		else
		{
			robot_control_data->mode.fric_cover_mode = cover_on_ENUM;
			Set_Beep_Time(4, 800, 60);
		}
	}

	//Ħ����ģʽ E
	if (KEY_CLICKED(KEY_E))
	{
		if (robot_control_data->mode.fric_cover_mode == fric_adaptive_speed_mode_ENUM)
		{
			robot_control_data->mode.fric_cover_mode = fric_cover_off_mode_ENUM;
			Set_Beep_Time(1, 800, 60);
		}
		else
		{
			robot_control_data->mode.fric_cover_mode = fric_adaptive_speed_mode_ENUM;
			Set_Beep_Time(2, 800, 60);
		}
	}

#undef KEY_CTRL
#undef KEY_SHIFT
#undef KEY_Z
#undef KEY_X
#undef KEY_C
#undef KEY_V
#undef KEY_B
#undef KEY_F
#undef KEY_G
#undef KEY_Q
#undef KEY_E
#undef KEY_R
#undef KEY_VALUE
#undef OLD_KEY_VALUE
#undef KEY_PRESSED
#undef KEY_CLICKED
}

/* ��Ӧ���ģʽ */
static void Shoot_Key_Control(Rc_Ctrl_t *rc_data, Rc_Ctrl_t *last_rc_data, Robot_control_data_t *robot_control_data)
{
	//ң����ģʽ
	if (robot_control_data->mode.control_device == remote_controller_device_ENUM)
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

	//����ģʽ
	else if (robot_control_data->mode.control_device == 1)
	{
		if (rc_data->mouse.press_l && (!last_rc_data->mouse.press_l))
		{
			robot_control_data->mode.shoot_key = 1;
		}
	}
}
