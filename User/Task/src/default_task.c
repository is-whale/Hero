#include "default_task.h"

void StartDefaultTask(void const *argument)
{
	Led_Flow_Flash(); //暂时放这

	/* 关闭所有LED */
	LED_GREEN_OFF();
	LED_RED_OFF();

	/* 炫酷的闪烁后关闭 */
	LED_GREEN_ON();
	osDelay(5);
	LED_GREEN_OFF();
	LED_RED_ON();
	osDelay(5);
	LED_RED_OFF();
	osDelay(5);
	LED_GREEN_ON();
	LED_RED_ON();
	osDelay(5);
	LED_GREEN_OFF();
	LED_RED_OFF();
	osDelay(5);
	LED_GREEN_ON();
	LED_RED_ON();
	osDelay(5);
	LED_GREEN_OFF();
	LED_RED_OFF();
	osDelay(5);

	for (;;)
	{
		LED_GREEN_TOGGLE();
		osDelay(500);
	}
}
