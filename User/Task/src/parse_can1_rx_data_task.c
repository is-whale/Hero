#include "parse_can1_rx_data_task.h"

extern osThreadId parseCan1RxDataHandle;
static const uint16_t can1_get_data_signal = 0x0001;
static const uint8_t can1_motor_device_number = 4;
static Motor_Measure_t m3508_feddback_data[can1_motor_device_number];

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
                debug_showdata1("mechine angle",m3508_feddback_data[0].mechanical_angle);
                debug_showdata1("speed",m3508_feddback_data[0].speed_rpm);
                debug_showdata1("temperate",m3508_feddback_data[0].temperate);
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

Motor_Measure_t *Get_Can1_Feedback_Data(void)
{
    return m3508_feddback_data;
}

const uint8_t *Get_Can1_Motor_DeviceNumber(void)
{
    return (&can1_motor_device_number);
}
