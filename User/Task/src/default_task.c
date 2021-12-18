#include "default_task.h"

void StartDefaultTask(void const *argument)
{
	/* 关闭红绿LED */
	LED_GREEN_OFF();
	LED_RED_OFF();

	Laser_ON();///<打开激光
	for (;;)
	{
		LED_GREEN_TOGGLE();
		osDelay(500);
	}
}
