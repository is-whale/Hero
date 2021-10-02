#include "buzzer_task.h"

extern osMessageQId buzzerQueueHandle;
extern osThreadId buzzerTaskHandle;
static Beep_Play_t *beep_play;

void StartBuzzerTask(void const *argument)
{
    osEvent buzzer_task_get_data_event;

    Buzzer_Init();
    LL_TIM_OC_SetCompareCH1(TIM12, 0);
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
 * @brief               蜂鸣器播报数据
 * @param beep_time     播报次数
 * @param pulse         占空比
 * @param delay_time    响应时间
 * @return osStatus     发送状态
 */
osStatus Set_Beep_Time(uint8_t beep_time, uint16_t pulse, uint16_t delay_time)
{
    static Beep_Play_t beep_msg;
    beep_msg.beep_time = beep_time;
    beep_msg.pulse = pulse;
    beep_msg.delay_time = delay_time;
    return osMessagePut(buzzerQueueHandle, (uint32_t)&beep_msg, 0);
}
