#include "default_task.h"

void StartDefaultTask(void const *argument)
{
	Led_Flow_Flash(); //��ʱ����

	/* �ر�����LED */
	LED_GREEN_OFF();
	LED_RED_OFF();

	/* �ſ����˸��ر� */
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
