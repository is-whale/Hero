#include "remote_task.h"

static Rc_Ctrl_t remote_controller;
static Rc_Ctrl_t *last_time_rc;
extern osThreadId remoteTaskHandle;
static uint16_t *sbus_rx_data_len;

static uint8_t *rc_rx_buffer[2];
static const uint32_t remote_get_data_signal = 0x0001;
static const uint32_t remote_heartbeat_time = 50;

void StartRemoteTask(void const *argument)
{
    static uint8_t rx_available_buffer_index;
    osEvent remote_get_data_event;

    rc_rx_buffer[0] = Get_Usart1_DMA_RxBuffer_One();
    rc_rx_buffer[1] = Get_Usart1_DMA_RxBuffer_Two();
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
            Parse_Remoter_Data(rc_rx_buffer[rx_available_buffer_index] , &remote_controller);
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
