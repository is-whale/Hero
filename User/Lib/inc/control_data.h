#ifndef __CONTROL_DATA_H_
#define __CONTROL_DATA_H_

#include "stm32f4xx.h"
#include "remote.h"

typedef struct
{
	struct
	{
		uint8_t control_device;				 //控制设备 1键鼠模式(rc) 2遥控器模式(mouse_key)
		uint8_t rc_motion_mode;				 //底盘云台模式 1底盘跟随 2小陀螺 3自瞄跟随 4自瞄小陀螺 5特殊
		uint8_t mouse_keyboard_chassis_mode; //1跟随 2小陀螺 3特殊
		uint8_t mouse_keyboard_gimbal_mode;	 //1手动 2自瞄 3特殊
		uint8_t shoot_mode;					 //射击模式 1单发 2连发（慢速） 3连发（最大速度）
		uint8_t fric_cover_mode;			 //摩擦轮模式 0关闭 1自适应 2最高速 3开舱盖

		//const鼠标最大速度
		int16_t mouse_x_max_value;
		int16_t mouse_y_max_value;

		//const鼠标数据倍率(灵敏度)
		float mouse_x_sensitivity;
		float mouse_y_sensitivity;

		//底盘加速度
		int16_t chassis_acceleration;
		//底盘最大速度
		int16_t chassis_max_speed; 

	} mode;

	struct
	{
		int16_t ch0; //鼠标
		int16_t ch1; //鼠标
		int16_t ch2; //左右 键盘WS
		int16_t ch3; //前后 键盘AD
	} virtual_rocker;

	//ctrl按下切换
	//陀螺开关、正反-1 0 1
	//归入地盘模式
	//或者单独一个小陀螺 正反

	//shift控制（按住）
	//速度、加速度倍率 x = 0~1
	//加速度倍率 1+x
	//速度倍率 1+x
	
	
}Robot_contral_data_t;

// void Parse_Robot_Contral_Data(Rc_Ctrl_t* rc_data, Rc_Ctrl_t* last_rc_data, Robot_contral_data_t* robot_contral_data);

#endif // __CONTROL_DATA_H_
