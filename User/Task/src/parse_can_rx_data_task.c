#include "parse_can_rx_data_task.h"

static const uint32_t can1_rx_get_data_signal = 0x0001;
static const uint32_t can2_rx_get_data_signal = 0x0002;
static const uint16_t can_rx_data_overtime = 500;

extern osThreadId parseCanRxDataTHandle;

void StartParseCanRxDataTask(void const *argument)
{
    osEvent can_get_data_event;

    osDelay(100);

    for (;;)
    {
        can_get_data_event = osSignalWait(can1_rx_get_data_signal | can2_rx_get_data_signal, can_rx_data_overtime);
        if (can_get_data_event.status == osEventSignal)
        {
            if (can_get_data_event.value.signals == can1_rx_get_data_signal)
            {
                // TODO CAN1 接收到数据
            }
            if (can_get_data_event.value.signals == can2_rx_get_data_signal)
            {
                // TODO CAN2 接收到数据
            }
        }
        osDelay(10);
    }
}

void Info_Parse_Can1_Data(void)
{
    osSignalSet(parseCanRxDataTHandle, can1_rx_get_data_signal);
}

void Info_Parse_Can2_Data(void)
{
    osSignalSet(parseCanRxDataTHandle, can2_rx_get_data_signal);
}
