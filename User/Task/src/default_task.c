#include "default_task.h"

void StartDefaultTask(void const *argument)
{
	/* ¹Ø±ÕºìÂÌLED */
	LED_GREEN_OFF();
	LED_RED_OFF();

	for (;;)
	{
		LED_GREEN_TOGGLE();
		osDelay(500);
	}
}
