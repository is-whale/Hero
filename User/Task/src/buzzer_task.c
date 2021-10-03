/**
 * @file buzzer_task.c
 * @brief ����������ʹ����Ϣ���У����Ǵ���Ĳ����ǽṹ��ָ�룬����ָ�����������Ľ�
 * @version 0.1
 * @date 2021-10-03
 * @copyright Copyright (c) 2021
 */

#include "buzzer_task.h"

extern osMessageQId buzzerQueueHandle; ///< ���������ݶ���
extern osThreadId buzzerTaskHandle;    ///< ������������
static Beep_Play_t *beep_play;         ///< ���������ݽṹ��ָ��

void StartBuzzerTask(void const *argument)
{
    osEvent buzzer_task_get_data_event;

    Buzzer_Init();

    osDelay(100);

    for (;;)
    {
        buzzer_task_get_data_event = osMessageGet(buzzerQueueHandle, osWaitForever);

        if (buzzer_task_get_data_event.status == osEventMessage)
        {
            beep_play = (Beep_Play_t *)buzzer_task_get_data_event.value.v;
            for (uint8_t i = 0; i < beep_play->beep_time; i++)
            {
                BUZZER_ON(beep_play->pulse);
                osDelay(beep_play->delay_time);

                BUZZER_OFF();
                osDelay(beep_play->delay_time);
            }
        }
    }
}

/**
 * @brief               ��������������
 * @param beep_time     ��������
 * @param pulse         ռ�ձ�
 * @param delay_time    ��Ӧʱ��
 * @return osStatus     ����״̬
 */
osStatus Set_Beep_Time(uint8_t beep_time, uint16_t pulse, uint16_t delay_time)
{
    static Beep_Play_t beep_msg;
    beep_msg.beep_time = beep_time;
    beep_msg.pulse = pulse;
    beep_msg.delay_time = delay_time;
    return osMessagePut(buzzerQueueHandle, (uint32_t)&beep_msg, 0);
}
