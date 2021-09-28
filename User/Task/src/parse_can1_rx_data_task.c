#include "parse_can1_rx_data_task.h"

extern osThreadId parseCan1RxDataHandle;
static const uint16_t can1_get_data_signal = 0x0001;
Motor_Measure_t m3508_feddback_data[4];

void StartParseCan1RxDataTask(void const *argument)
{
    osEvent can1_get_data_event;

    Can1_Filter_Init();

    osDelay(100);

    for (;;)
    {
        can1_get_data_event = osSignalWait(can1_get_data_signal, can1_get_data_signal);
        if (can1_get_data_event.status == osEventSignal)
        {
            if (can1_get_data_event.value.signals == can1_get_data_signal)
            {
                Parse_Can1_Rxd_Data();
            }
        }
        else if (can1_get_data_event.status == osEventTimeout)
        {
            // can1 接收数超时处理
        }
    }
}

void Parse_Can1_Rxd_Data(void)
{
    CAN_RxHeaderTypeDef can1_rx_header;
    static uint8_t i = 0;
    static uint8_t can1_rx_data[8];
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &can1_rx_header, can1_rx_data);
    i = can1_rx_header.StdId - CAN_3508_M1_ID;
    GET_MOTOR_MEASURE_DATA(&m3508_feddback_data[i], can1_rx_data);
}

void Info_Can1_ParseData_Task(void)
{
    osSignalSet(parseCan1RxDataHandle, can1_get_data_signal);
}
