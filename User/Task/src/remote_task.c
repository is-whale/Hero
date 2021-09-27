#include "remote_task.h"

static Rc_Ctrl_t remote_controller;
static Rc_Ctrl_t *last_time_rc;
extern osThreadId remoteTaskHandle;
static uint16_t *sbus_rx_data_len;

static uint8_t *rc_rx_buffer_one;
static uint8_t *rc_rx_buffer_two;

static const uint32_t remote_get_data_signal = 0x0001;
static const uint32_t remote_heartbeat_time = 50;

void StartRemoteTask(void const *argument)
{
    static uint8_t rx_available_buffer_index;
    osEvent remote_get_data_event;
    rc_rx_buffer_one = Get_Usart1_DMA_RxBuffer_One();
    rc_rx_buffer_two = Get_Usart1_DMA_RxBuffer_Two();
    sbus_rx_data_len = Get_Usart1_DMA_Rxd_DataLen();

    Usart1_RxDMA_Init();

    Rc_Data_Reset(&remote_controller);
    Rc_Data_Reset(last_time_rc);

    osDelay(100);

    for (;;)
    {
        remote_get_data_event = osSignalWait(remote_get_data_signal, remote_heartbeat_time);
        if (remote_get_data_event.status == osEventSignal)
        {
            rx_available_buffer_index = Get_Rc_Available_Bufferx();
            debug_print("rx len:%d\r\n", *sbus_rx_data_len);
            Parse_Remoter_Data(rc_rx_buffer_one, &remote_controller);
            debug_print("rc_ch0 %d\r\n", remote_controller.rc.ch0);
            debug_print("rc_ch1 %d\r\n", remote_controller.rc.ch1);
            debug_print("rc_ch2 %d\r\n", remote_controller.rc.ch2);
            debug_print("rc_ch3 %d\r\n", remote_controller.rc.ch3);
            osDelay(500);
        }
        else if (remote_get_data_event.status == osEventTimeout)
        {
            Rc_Data_Reset(&remote_controller);
            Rc_Data_Reset(last_time_rc);
        }
    }
}

void Info_RemoteTask_Parse_Data(void)
{
    osSignalSet(remoteTaskHandle, remote_get_data_signal);
}
