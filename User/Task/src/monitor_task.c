#include "monitor_task.h"

static Module_status_t module_status[8]; ///< 模块数量

static void Monitor_Task_Init(void);   ///< 初始化监听任务
static void Monitor_All_Modules(void); ///< 执行监听

void StartMonitorTask(void const *argument)
{
	Monitor_Task_Init(); ///< 监听任务初始化
	Led_Flow_Flash();	 ///< 初始化流水灯
	Buzzer_Bsp_Init();	 ///< 初始化时蜂鸣

	osDelay(50);

	for (;;)
	{
		Monitor_All_Modules();

		osDelay(200);
	}
}

/**
 * @brief 		获取指定模块的在线状态
 * @param 		{uint8_t} id moduleList的枚举成员
 * @return		0 & 1 在线状态
 */
uint8_t Get_Module_Online_State(uint8_t id)
{
	return Get_Module_Status(&module_status[id]);
}

/**
 * @brief 	模块状态重置
 * @param 	{uint8_t} id moduleList的枚举成员
 * @return	 {*}
 */
void Module_Reload(uint8_t index)
{
	Module_Status_Reload(&module_status[index]);
}
/**
 * @brief 	底盘重装载
 * @param 	{int8_t} 电机ID
 * @return	 {*}
 */
void Classis_Reload(int8_t motor_index)
{
	static uint8_t chassis_motor[4] = {0, 0, 0, 0};

	if (motor_index >= 0 && motor_index <= 3)
	{
		chassis_motor[motor_index] = 1;
	}
	else
	{
		return;
	}
	if (chassis_motor[0] && chassis_motor[1] && chassis_motor[2] && chassis_motor[3])
	{
		chassis_motor[0] = 0;
		chassis_motor[1] = 0;
		chassis_motor[2] = 0;
		chassis_motor[3] = 0;
		Module_Status_Reload(&module_status[1]);
	}
}
/**
 * @brief: 云台重装载
 * @param {int8_t} motor_index must be 1 or 2. 为1时重装载yaw轴电机，2装载pitch电机。
 * @return {*}
 */
void Gimbal_Reload(int8_t motor_index)
{
	// bit0代表电机yaw bit1代表电机pitch
	static uint8_t gimbal_motor_state = 0;

	if (motor_index >= 1 && motor_index <= 2)
	{
		gimbal_motor_state |= (0x01 << (motor_index - 1));
	}
	else
	{
		return;
	}
	if ((gimbal_motor_state & 0x03) == 0x03)
	{
		gimbal_motor_state = 0;
		Module_Status_Reload(&module_status[gimbal_motor]);
	}
}

/**
 * @brief 初始化监听任务
 * @param {*}
 * @return {*}
 */
static void Monitor_Task_Init(void)
{
	for (uint8_t i = 0; i < 8; i++)
	{
		Module_Status_Init(&module_status[i], 5, NULL, NULL);
	}
}

/**
 * @brief 执行监听
 * @param {*}
 * @return {*}
 */
static void Monitor_All_Modules(void)
{
#define MODULE_REONLINE(index) (module_status[index].time_out_flag == 0 && module_status[index].old_time_out_flag == 1)
#define MODULE_OFFLINE(index) (module_status[index].time_out_flag == 1 && module_status[index].old_time_out_flag == 0)

	for (uint8_t i = 0; i < 8; i++)
	{
		//各个模块状态刷新
		Module_Status_Listen(&module_status[i]);

		if (MODULE_REONLINE(i))
		{
			LED_FLOW_ON(i);
			info_log("Module%d Online.\r\n\r\n", i);
		}
		else if (MODULE_OFFLINE(i))
		{
			LED_FLOW_OFF(i);
			info_log("Module%d Offline.\r\n\r\n", i);
		}
	}
}

#undef MODULE_OFFLINE
#undef MODULE_REONLINE
