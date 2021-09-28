#include "parse_can1_rx_data_task.h"

extern osThreadId parseCan1RxDataHandle;
static const uint16_t can1_get_data_signal = 0x0001;

void StartParseCan1RxDataTask(void const *argument)
{
    osEvent can1_get_data_event;

    osDelay(100);

    for (;;)
    {
        can1_get_data_event = osSignalWait(can1_get_data_signal,can1_get_data_signal);
        if (can1_get_data_event.status == osEventSignal)
        {
            if (can1_get_data_event.value.signals == can1_get_data_signal)
            {
                // TODO 解析 can1 接收的数据
            }
        }
        else if (can1_get_data_event.status == osEventTimeout)
        {
            // can1 接收数超时处理
        }
    }
}

void Info_Can1_ParseData_Task()
{
    osSignalSet(parseCan1RxDataHandle,can1_get_data_signal);
}