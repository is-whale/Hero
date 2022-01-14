#include "referee_system.h"

static uint8_t *referee_system_rx_buffer[2];
static uint16_t *referee_system_rxd_len;
static const uint32_t referee_system_get_data_signal = 0x0001; ///< ����ϵͳ���ܵ��������ź�
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

                if (1)///<�������ݲ�����
                {
                    Console.print("%s\r\n", referee_system_rx_buffer[referee_system_rx_buffer_index]);
                    LL_USART_TransmitData8(USART6, 9);
                    // Parse Referee System Data
                    //Module_Reload(); ///< ���²���ϵͳ״̬
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
 * @brief           ֪ͨ����ϵͳ������н������ڴ��� 6 ���жϺ����н��е���
 * @param[in]       none
 * @retval          void
 */
void Info_Referee_System_Task_Parse_Data(void)
{
    osSignalSet(refereeSystemTaHandle, referee_system_get_data_signal);
}
