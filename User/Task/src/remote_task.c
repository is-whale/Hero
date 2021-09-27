#include "remote_task.h"

static uint8_t *sbus_rx_buffer[2];
static Rc_Ctrl_t *remote_controller;
static Rc_Ctrl_t *last_time_rc;

void StartRemoteTask(void const *argument)
{
    static uint8_t rx_available_buffer_index;
    sbus_rx_buffer[0] = Get_Usart1_DMA_RxBuffer_One();
    sbus_rx_buffer[1] = Get_Usart1_DMA_RxBuffer_Two();

    Rc_Data_Reset(remote_controller);
    Rc_Data_Reset(last_time_rc);

    osDelay(100);

    for (;;)
    {
        if (1)
        {
            rx_available_buffer_index = Get_Rc_Available_Bufferx();
            Parse_Remoter_Data(sbus_rx_buffer[rx_available_buffer_index], remote_controller);
        }
        else
        {
            Rc_Data_Reset(remote_controller);
            Rc_Data_Reset(last_time_rc);
        }
        osDelay(10);
    }
}

void Info_RemoteTask_Parse_Data(void)
{
	
}
