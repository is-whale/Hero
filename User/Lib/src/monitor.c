#include "monitor.h"

/**
 * @brief ģ��״̬��ʼ��
 * @param {Module_status_t*} module ģ��ṹ���ָ��
 * @param {uint8_t} cnt_num ģ�鳬ʱʱ�䣨ʵ��ʱ��Ϊ cnt_num*������ʱʱ�䣩
 * @param {void} Online_Callback ģ�����ߵĻص�����
 * @param {void} Offline_Callback ģ�����ߵĻص�����
 * @return {*}
 */
void Module_Status_Init(Module_status_t* module, uint8_t cnt_num, void Online_Callback(void), void Offline_Callback(void))
{
	module->count = 0;
	module->reload_cnt = cnt_num;
	
	module->time_out_flag = 1;
	module->old_time_out_flag = 1;
	
	module->Module_Online_Callback = Online_Callback;
	module->Module_Offline_Callback = Offline_Callback;
}

/**
 * @brief ģ��������ڶ�ʱ���������ж�ʱ����
 * @param {Module_status_t*} module ģ��ṹ���ָ��
 * @return {*}
 */
void Module_Status_Listen(Module_status_t* module)
{
	module->old_time_out_flag = module->time_out_flag;
	
	if(module->count > 0)
	{
		module->count--;
		module->time_out_flag=0;
		if(module->Module_Online_Callback!=NULL && module->old_time_out_flag == 1)
		{
			module->Module_Online_Callback();
		}
	}
	else if(module->count == 0)
	{
		module->time_out_flag=1;
		if(module->Module_Offline_Callback!=NULL && module->old_time_out_flag == 0)
		{
			module->Module_Offline_Callback();
		}
	}
}

/**
 * @brief �õ�ģ��״̬
 * @param {Module_status_t*} module ģ��ṹ���ָ��
 * @return {*}
 */
uint8_t Get_Module_Status(Module_status_t* module)
{
	//return module->time_out_flag;
	if(module->time_out_flag == 0)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

/**
 * @brief ģ��״̬���ã�������ģ���յ�����ʱ���ã�
 * @param {Module_status_t*} module ģ��ṹ���ָ��
 * @return {*}
 */
void Module_Status_Reload(Module_status_t* module)
{
	module->count = module->reload_cnt;
}

