#include "monitor_task.h"

static Module_status_t module_status[8];

static void Monitor_Task_Init(void);
static void Monitor_All_Modules(void);

void StartMonitorTask(void const *argument)
{
	Monitor_Task_Init();
	Led_Flow_Flash();

	osDelay(50);

	for (;;)
	{
		Monitor_All_Modules();

		osDelay(200);
	}
}

/**
 * @brief 获取指定模块的在线状态
 * @param {uint8_t} id moduleList的枚举成员
 * @return {*}
 */
uint8_t Get_Module_Online_State(uint8_t id)
{
	return Get_Module_Status(&module_status[id]);
}

/**
 * @brief 获取指定模块的在线状态
 * @param {uint8_t} id moduleList的枚举成员
 * @return {*}
 */
void Module_Reload(uint8_t index)
{
	Module_Status_Reload(&module_status[index]);
}

/**
 * @brief 重装载电机状态
 * @param {int8_t} motor_index 参数必须为1~4，分别对应底盘4个电机
 * @return {*}
 */
void Classis_Reload(int8_t motor_index)
{
	//bit0代表电机1 bit1代表电机2 bit2代表电机3 bit3代表电机4
	static uint8_t chassis_motor_state = 0;
	if(motor_index>=1 && motor_index<=4)
	{
		chassis_motor_state |= (0x01 << (motor_index-1));
	}
	else
	{
		return;
	}
	if((chassis_motor_state & 0x0F) == 0x0F)
	{
		chassis_motor_state = 0;
		Module_Status_Reload(&module_status[chassis_motor]);
	}
}

/**
 * @brief: 云台重装载
 * @param {int8_t} motor_index must be 1 or 2. 为1时重装载yaw轴电机，2装载pitch电机。
 * @return {*}
 */
void Gimbal_Reload(int8_t motor_index)
{
	//bit0代表电机yaw bit1代表电机pitch
	static uint8_t gimbal_motor_state = 0;
	
	if(motor_index>=1 && motor_index<=2)
	{
		gimbal_motor_state |= (0x01 << (motor_index-1));
	}
	else
	{
		return;
	}
	if((gimbal_motor_state & 0x03) == 0x03)
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
	for(uint8_t i=0; i<8; i++)
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

#undef MODULE_OFFLINE
#undef MODULE_REONLINE
}


