#include "default_task.h"

void StartDefaultTask(void const *argument)
{
	/* �رպ���LED */
	LED_GREEN_OFF();
	LED_RED_OFF();

	Laser_ON(); ///<�򿪼���
	for (;;)
	{
		LED_GREEN_TOGGLE();//��ɫLED�л��������ж�ϵͳ�Ƿ��������������0.5s��תһ��
		osDelay(500);
	}
}
