#include "referee_system.h"

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
�������system_device.c��
*/
#if 0
	#define JUDGE_LOG DEBUG_LOG
	#define JUDGE_ERROR DEBUG_ERROR
#else
	#define JUDGE_LOG(format, arg...) /* DEBUG OFF */
	#define JUDGE_ERROR(err) /* DEBUG OFF */
#endif
/*��������*/
//uint8_t Parse Referee System Data(u8 *get_data, u16 data_len);///<����ϵͳ���ڽ�����������


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
                    Console.print("%s\r\n", referee_system_rx_buffer[referee_system_rx_buffer_index]);
                    LL_USART_TransmitData8(USART6, 9);
                    /*�����ܵĵ�ԭʼ���ݿ���Ϊ����*/
		            memcpy(judge_buf_copy, judge_buf, judge_buf_len);
                    
                    Module_Reload(judge_system); ///< ���²���ϵͳ״̬

                //    Parse Referee System Data(judge_buf_copy, judge_buf_len);
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

/*
  ��������Analysis_Judge_System
  ����  ����������ϵͳ����
  ����  ��get_data��Ҫ������֡ͷ���ݣ�data_len���ݳ���
  ����ֵ��0--����ʧ�� 1--�����ɹ�
*/
// uint8_t Parse Referee System Data(u8 *get_data, u16 data_len)
// {
// 	u8 a5_position[8]; //0xA5��λ��
// 	u8 a5_number = 0;  //0xA5�ĸ��������ݰ�������
// 	u16 data_length[8];  //ÿ��data���ݰ��ĳ���
	
// 	//Ѱ��֡ͷ
// 	Find_All_A5(get_data, data_len, a5_position, &a5_number);
	
// 	//�����������ݰ�
// 	for(u8 i=0; i<a5_number; i++)
// 	{
// 		//����֡ͷ����
// 		if( Analysis_Frame_Header(&get_data[ (a5_position[i]) ], &data_length[i], NULL) == 0)
// 		{
// 			JUDGE_LOG("Analysis No.%d frame header error.", i);
// 			continue;
// 		}
		
// 		//����CRC16У��
// 		if( Check_Package_Crc16(&get_data[ (a5_position[i]) ], (data_length[i]+9)) == 0 )
// 		{
// 			JUDGE_LOG("CRC16 check No.%d fail.", i);
// 			continue;
// 		}
		
// 		//���������ݰ�
// 		if( Analysis_Data(&get_data[ (a5_position[i]) ], data_length[i]) == 0)
// 		{
// 			JUDGE_LOG("Analysis No.%d data fail.", i);
// 			continue;
// 		}
		
// 		// DEBUG_PRINT("x%d len:%d p:%d id:%d\r\n", i, data_length[i], a5_position[i], Analysis_Cmd_Id( &get_data[ (a5_position[i]) ] ) );
		
		
// 	}
	
// 	return 1;
// }

