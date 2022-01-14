#include "referee_system.h"

static uint8_t *referee_system_rx_buffer[2];
static uint16_t *referee_system_rxd_len;
static const uint32_t referee_system_get_data_signal = 0x0001; ///< 裁判系统接受到的任务信号
static uint8_t referee_system_rx_buffer_index;
static const uint32_t referee_system_data_overtime = 50;  
extern osThreadId refereeSystemTaHandle;   

void StartRefereeSystemTask(void const *argument)
{
    osEvent referee_system_get_data_event;
    Usart6_RxDMA_Init();

    referee_system_rx_buffer[0] = Get_Usart6_DMA_RxBuffer_One();
    referee_system_rx_buffer[1] = Get_Usart6_DMA_RxBuffer_Two();
    referee_system_rxd_len = Get_Usart6_DMA_Rxd_DataLen();

    osDelay(10);

    for (;;)
    {
        referee_system_get_data_event = osSignalWait(referee_system_get_data_signal, referee_system_data_overtime);
        if (referee_system_get_data_event.status == osEventSignal)
        {
            if (referee_system_get_data_event.value.signals == referee_system_get_data_signal)
            {
                referee_system_rx_buffer_index = Get_USART6_DMA_Available_Bufferx();

                if (1)///<解析数据并检验
                {
                    Console.print("%s\r\n", referee_system_rx_buffer[referee_system_rx_buffer_index]);
                    LL_USART_TransmitData8(USART6, 9);
                    // Parse Referee System Data
                    //Module_Reload(); ///< 更新裁判系统状态
                }
                else
                {
                    Usart6_Rx_DMA_Reset();
                }
            }
        }
        else if (referee_system_get_data_event.status == osEventTimeout)
        {
        }

        osDelay(10);
    }
}
/**
 * @brief           通知裁判系统任务进行解析，在串口 6 的中断函数中进行调用
 * @param[in]       none
 * @retval          void
 */
void Info_Referee_System_Task_Parse_Data(void)
{
    osSignalSet(refereeSystemTaHandle, referee_system_get_data_signal);
}
