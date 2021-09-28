#include "remote_task.h"

static Rc_Ctrl_t remote_controller; ///< 本次解析的遥控器数据
static Rc_Ctrl_t last_time_rc;      ///< 上次解析的遥控器数据
extern osThreadId remoteTaskHandle; ///< 遥控器任务句柄
static uint16_t *sbus_rxd_len;
static uint8_t *rc_rx_buffer[2];                       ///< 遥控器两次接收数据缓冲
static const uint32_t remote_get_data_signal = 0x0001; ///< 遥控器接收数据信号
static const uint32_t remote_heartbeat_time = 50;      ///< 遥控器数据包心跳时间

void StartRemoteTask(void const *argument)
{
    static uint8_t rx_available_buffer_index;
    osEvent remote_get_data_event;

    rc_rx_buffer[0] = Get_Usart1_DMA_RxBuffer_One();
    rc_rx_buffer[1] = Get_Usart1_DMA_RxBuffer_Two();
    sbus_rxd_len = Get_Usart1_DMA_Rxd_DataLen();

    Usart1_RxDMA_Init();

    Rc_Data_Reset(&remote_controller);
    Rc_Data_Reset(&last_time_rc);

    osDelay(100);

    for (;;)
    {
        remote_get_data_event = osSignalWait(remote_get_data_signal, remote_heartbeat_time);
        if (remote_get_data_event.status == osEventSignal)
        {
            if (remote_get_data_event.value.signals == remote_get_data_signal)
            {
                rx_available_buffer_index = Get_Rc_Available_Bufferx();

                if (Rc_Data_Check_Parse(rc_rx_buffer[rx_available_buffer_index], &remote_controller, *sbus_rxd_len))
                {
                    Module_Reload(remote_control);
                    // TODO
                }
                else
                {
                    // TODO
                }
            }
        }
        else if (remote_get_data_event.status == osEventTimeout)
        {
            Rc_Data_Reset(&remote_controller);
            Rc_Data_Reset(&last_time_rc);
        }
    }
}

/**
 * @brief           通知 remoteTask 进行遥控器数据的解析，在 USART1 的串口中断中调用
 * @param[in]       none
 * @retval          void
 */
void Info_RemoteTask_Parse_Data(void)
{
    osSignalSet(remoteTaskHandle, remote_get_data_signal);
}
