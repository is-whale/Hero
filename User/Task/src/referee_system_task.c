#include "referee_system_task.h"


static uint8_t *referee_system_rx_buffer[2];
static uint16_t *referee_system_rxd_len;
static const uint32_t referee_system_get_data_signal = 0x0001; ///< ����ϵͳ���ܵ��������ź�
static uint8_t referee_system_rx_buffer_index;
static const uint32_t referee_system_data_overtime = 50;  
extern osThreadId refereeSystemTaHandle;   

static const uint8_t *judge_buf;
static uint8_t judge_buf_copy[128];
static uint32_t judge_buf_len = 0;


/* ����ϵͳ���Ժ궨�� 
��������system_device.c��
*/
#if 0
	#define DEBUG_LOG debug_log
	#define DEBUG_ERROR debug_error
#else
	#define DEBUG_LOG(format, arg) /* DEBUG OFF */
	#define DEBUG_ERROR(err) /* DEBUG OFF */
#endif
/*��������*/
//uint8_t Parse Referee System Data(uint8_t *get_data, uint16_t data_len);///<����ϵͳ���ڽ�����������


void StartRefereeSystemTask(void const *argument)
{
    osEvent referee_system_get_data_event;
    Usart6_RxDMA_Init();

    referee_system_rx_buffer[0] = Get_Usart6_DMA_RxBuffer_One();
    referee_system_rx_buffer[1] = Get_Usart6_DMA_RxBuffer_Two();
    referee_system_rxd_len = Get_Usart6_DMA_Rxd_DataLen();
	
    /*��������*/
	
    (void)referee_system_rxd_len;///<����warning

    /*�����������*/
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
                    /*������յ��Ĳ���ϵͳ����*/
                    Console.print("%s\r\n", referee_system_rx_buffer[referee_system_rx_buffer_index]);

                    LL_USART_TransmitData8(USART6, 9);
                    /*�����ܵĵ�ԭʼ���ݿ���Ϊ����*/
		            memcpy(judge_buf_copy, judge_buf, judge_buf_len);
                    
                    Module_Reload(judge_system); ///< ���²���ϵͳ״̬

                  Parse_Refere_System_Data(judge_buf_copy, judge_buf_len);
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




