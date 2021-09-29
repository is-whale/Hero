#include "control_data.h"

static void Robot_Rc_Mode_Change_Control(Rc_Ctrl_t* rc_data, Rc_Ctrl_t* last_rc_data, Robot_control_data_t* robot_control_data);
static void Wasd_Key_To_Virtual_Rocker(Rc_Ctrl_t* rc_data, Robot_control_data_t* robot_control_data);

void Parse_Robot_Control_Data(Rc_Ctrl_t* rc_data, Rc_Ctrl_t* last_rc_data, Robot_control_data_t* robot_control_data)
{
	Robot_Rc_Mode_Change_Control(rc_data, last_rc_data, robot_control_data); ///< 机器人模式变换响应
	Wasd_Key_To_Virtual_Rocker(rc_data, robot_control_data); //WASD虚拟为摇杆

	/* 鼠标虚拟为摇杆 */
	//TODO

	/* 响应键盘控制 */
	//TODO

	/* 射击按钮 */
	//TODO
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

	/* 操控设备选择 */
	if(S2_CHANGED_TO(3,2))
	{
		ROBOT_MODE.control_device++;
		if(ROBOT_MODE.control_device==3) ROBOT_MODE.control_device=1;
		// Set_Beep_Time(ROBOT_MODE.control_device, 1000, 55);
	}
	
	/* 底盘云台模式 */
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

	/* 摩擦轮速度模式，舱盖开关 */
	if(S1_CHANGED_TO(3,2))
	{
		ROBOT_MODE.fric_cover_mode++;
		if(ROBOT_MODE.fric_cover_mode==4) ROBOT_MODE.fric_cover_mode=0;
		// Set_Beep_Time(ROBOT_MODE.fric_cover_mode+1, 800, 60);
	}

	/* 射击模式选择器 */
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
	/*只按下W时，不按下S*/
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
	/*只按下S时，不按下W*/
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
	/*W、S都未按下时*/
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

	/*只按下D时*/
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
	/*只按下A时*/
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
	/*A、D都未按下时*/
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

