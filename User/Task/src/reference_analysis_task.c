#include "reference_analysis_task.h"

extern osThreadId ReferenceTaskHandle;

static const int32_t reference_task_get_date_signal = 0x000000ff;
static uint8_t uart8_rx_available_buffer_index; //< ����˫�����пɶ�ȡ������[0&1����Ϊ�����±�]
static uint8_t *reference_system_rxd_buffer[2]; //< ����ϵͳ˫�����������
static uint16_t *rxd_data_len;                  //< ����ϵͳ�������ݳ���
static uint8_t date_copy[128];                  ///< �����յ�������
const Judge_data_t *juder_date_finish;          ///< ��ʱ�洢������Ĳ���ϵͳ���ݣ�����ʹ��
void StartReferenceTaskTask(void const *argument)
{
    osEvent reference_get_data_event;                              ///< ֪ͨ����
    Uart8_Rx_Dma_Init();                                           ///< DMA��ʼ��
    rxd_data_len = Get_Uart8_DMA_Rxd_DataLen();                    ///< ָ����ճ��ȣ�����ʹ�ã�
    reference_system_rxd_buffer[0] = Get_Uart8_DMA_RxBuffer_One(); ///< ָ��˫��������
    reference_system_rxd_buffer[1] = Get_Uart8_DMA_RxBuffer_Two(); ///< ָ��˫��������
    juder_date_finish = Get_Referee_Data();                        //< ָ������������
    osDelay(100);
    for (;;)
    {
        reference_get_data_event = osSignalWait(reference_task_get_date_signal, osWaitForever); ///< �ȴ������жϵ�֪ͨ�ź�
        if (reference_get_data_event.status == osEventSignal)
        {
            if (reference_get_data_event.value.signals == reference_task_get_date_signal) ///< �յ��źţ���ʼ����
            {
                uart8_rx_available_buffer_index = Get_Reference_Available_Bufferx();
                memcpy(date_copy, reference_system_rxd_buffer[uart8_rx_available_buffer_index], *rxd_data_len); ///< ����ԭʼ����

                debug_showdata1("judge_buf_len", *rxd_data_len); ///< ��ӡ���ݳ���
                                                                 /**
                 * @brief   ��ӡԭʼ����
                 */
                for (uint8_t i = 0; i < *rxd_data_len; i++)
                {
                    debug_print("%d ", date_copy[i]);
                }
                debug_print("\r\n");

                Parse_Refere_System_Data(date_copy, *rxd_data_len); ///<��������ϵͳ����
                Module_Reload(judge_system);                        ///< ����ϵͳ���߼��
            }
        }
        else if (reference_get_data_event.status == osEventTimeout)
        {
            // TODO: δ��ʱ���յ�����ϵͳ���ݣ�������Ӧ�Ĵ�������˵���ߴ���
        }
        osDelay(200);
    }
}
/**
 * @brief       ֪ͨ����ϵͳ��������ʼ����
 * @note        ��uart8�жϺ����е���
 */
void Inform_Reference_Analysis_Task_Pasre_Data(void)
{

    osSignalSet(ReferenceTaskHandle, reference_task_get_date_signal);
}
