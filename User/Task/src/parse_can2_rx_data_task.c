#include "parse_can2_rx_data_task.h"

extern osThreadId parseCan2RxDataHandle;
static const uint16_t can2_get_data_signal = 0x0001;

void StartParseCan2RxDataTask(void const *argument)
{
    osEvent can2_get_data_event;

    osDelay(100);

    for (;;)
    {
        can2_get_data_event = osSignalWait(can2_get_data_signal,can2_get_data_signal);
        if (can2_get_data_event.status == osEventSignal)
        {
            if (can2_get_data_event.value.signals == can2_get_data_signal)
            {
                // TODO ���� can2 ���յ�����
            }
        }
        else if (can2_get_data_event.status == osEventTimeout)
        {
            // can2 ��������ʱ����
        }
    }
}

void Info_Can2_ParseData_Task(void)
{
    osSignalSet(parseCan2RxDataHandle,can2_get_data_signal);
}
