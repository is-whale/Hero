#include "super_capacitor_task.h"

static CAN_RxHeaderTypeDef *can1_rx_header_t;
static uint8_t *can1_rxd_data;
static int32_t super_capacitor_get_data_signal = 0x00000001;
static Super_Capacitor_t super_capacitor_t;

extern osThreadId superCapacitorTHandle;

void StartSuperCapacitorTask(void const *argument)
{
    osEvent super_capacitor_get_data_event;
    can1_rx_header_t = Get_CAN1_Rx_Header();
    can1_rxd_data = Get_CAN1_Rxd_Data();

    osDelay(100);

    for (;;)
    {
        super_capacitor_get_data_event = osSignalWait(super_capacitor_get_data_signal, osWaitForever);

        if (super_capacitor_get_data_event.status == osEventSignal)
        {
            if (super_capacitor_get_data_event.value.signals == super_capacitor_get_data_signal)
            {
                Parse_Super_Capacitor_RxdData(can1_rx_header_t, &super_capacitor_t, can1_rxd_data);
            }
        }
    }
}

void Parse_Super_Capacitor_RxdData(CAN_RxHeaderTypeDef *can1_rx_header_t, Super_Capacitor_t *cap_data, uint8_t *can_message)
{
    cap_data->input_voltage = (float)(can_message[1] << 8 | can_message[0]) / 100.f;
    cap_data->cap_voltage = (float)(can_message[3] << 8 | can_message[2]) / 100.f;
    cap_data->input_current = (float)(can_message[5] << 8 | can_message[4]) / 100.f;
    cap_data->target_power = (float)(can_message[7] << 8 | can_message[6]) / 100.f;
}

void Info_Super_Capacitor_Parse_Data(void)
{
    osSignalSet(superCapacitorTHandle, super_capacitor_get_data_signal);
}
