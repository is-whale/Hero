// #include "referee_system_task.h"
// #include "referee_system.h"

// static uint8_t *referee_system_rx_buffer[2];
// static uint16_t *referee_system_rxd_len;                       ///<����ϵͳ�������ݳ���
// static const uint32_t referee_system_get_data_signal = 0x0001; ///< ����ϵͳ���ܵ��������ź�
// static uint8_t referee_system_rx_buffer_index;
// // static const uint32_t referee_system_data_overtime = 50;//ʹ����ֱ������֪ͨ����ʱ����Ҫ
// extern osThreadId refereeSystemTaHandle;

// // static const uint8_t *judge_buf;
// static uint8_t judge_buf_copy[128];
// static uint32_t judge_buf_len = 0;
// void Referee_Init(void);                                                       ///<����ϵͳ��ʼ����������
// extern uint8_t Parse_Refere_System_Data(uint8_t *get_data, uint16_t data_len); ///< bug_log������ʼ�ձ�ʶ��Ϊ����
// const Judge_data_t *juder_date_after_parse;                                    ///< ������Ĳ���ϵͳ����ָ��
// /**
//  * @brief    ����ϵͳ���Ժ궨��,1Ϊ��
//  * @note     �������config.h��
//  */
// #if 0
// #define DEBUG_LOG debug_log
// #define DEBUG_ERROR debug_error
// #else
// #define DEBUG_LOG(format, arg) /* DEBUG OFF */
// #define DEBUG_ERROR(err)       /* DEBUG OFF */
// #endif
// /*��������*/
// // uint8_t Parse Referee System Data(uint8_t *get_data, uint16_t data_len);///<����ϵͳ���ڽ�����������

// void StartRefereeSystemTask(void const *argument)
// {
//     // osEvent referee_system_get_data_event;
//     // Referee_Init();
//     Usart6_RxDMA_Init();

//     referee_system_rx_buffer[0] = Get_Usart6_DMA_RxBuffer_One();
//     referee_system_rx_buffer[1] = Get_Usart6_DMA_RxBuffer_Two();
//     referee_system_rxd_len = Get_Usart6_DMA_Rxd_DataLen();
//     juder_date_after_parse = Get_Referee_Data();
//     referee_system_rx_buffer_index = Get_USART6_DMA_Available_Bufferx(); ///< ���õ�DMA����
//     /*��������*/
//     // (void)referee_system_rxd_len; ///<����warning
//     /*�����������*/
//     osDelay(10);
//     for (;;)
//     {
//         // referee_system_get_data_event = osSignalWait(referee_system_get_data_signal, referee_system_data_overtime);
//         xTaskNotifyWait(0x00, 0xFFFFFFFF, &judge_buf_len, portMAX_DELAY);
//         //   xTaskNotifyWait(0x00,0xffffffff, &judge_buf_len,0xffff);
//         // if (referee_system_get_data_event.status == osEventSignal)
//         // {
//         // if (referee_system_get_data_event.value.signals == referee_system_get_data_signal)
//         // {
//         // referee_system_rx_buffer_index = Get_USART6_DMA_Available_Bufferx();

//         if (1) ///<�������ݲ�����
//         /* debug_printf�趨��DEBUG����ʹ�� */
//         {
//             LL_USART_TransmitData8(USART6, 9);
//             /*�����ܵĵ�ԭʼ���ݿ���Ϊ����*/
//             memcpy(judge_buf_copy, referee_system_rx_buffer[referee_system_rx_buffer_index], judge_buf_len);
//             // debug_showdata1("judge_buf_len", judge_buf_len);
//             // for (uint8_t i = 0; i < judge_buf_len; i++)
//             // {
//             //     debug_print("%d ", judge_buf_copy[i]);
//             // }
//             // debug_print("\r\n");
//             Module_Reload(judge_system);                             ///< ���²���ϵͳ״̬
//             // Parse_Refere_System_Data(judge_buf_copy, judge_buf_len); ///<��������ϵͳ����

//             // Console.print("%0.2f\r\n", juder_date_after_parse ->power_heat_data.shooter_id1_17mm_cooling_heat);
//             // Console.print("%0.2f,0.2f\r\n", referee_system_rxd_len, juder_date_after_parse);
//         }
//         else
//         {
//             Usart6_Rx_DMA_Reset();
//         }
//         Usart6_Rx_DMA_Reset();
//         // }
//         // }
//         // else if (referee_system_get_data_event.status == osEventTimeout)
//         // {
//         // }

//         osDelay(10);
//     }
// }
// /**
//  * @brief       ����ϵͳ��ʼ��
//  * @param[in]   void
//  * @return      void
//  */
// void Referee_Init(void)
// {
//     Usart6_RxDMA_Init();

//     referee_system_rx_buffer[0] = Get_Usart6_DMA_RxBuffer_One();
//     referee_system_rx_buffer[1] = Get_Usart6_DMA_RxBuffer_Two();
//     referee_system_rxd_len = Get_Usart6_DMA_Rxd_DataLen();
//     juder_date_after_parse = Get_Referee_Data();
// }

// /**
//  * @brief           ֪ͨ����ϵͳ������н������ڴ��� 6 ���жϺ����н��е���
//  * @param[in]       none
//  * @retval          void
//  */
// void Info_Referee_System_Task_Parse_Data(void)
// {
//     osSignalSet(refereeSystemTaHandle, referee_system_get_data_signal);
// }
// /**
//  * @brief   ֪ͨ����ϵͳ������н���
//  */
// void Inform_Referee_System_Task_With_len(uint32_t send_date)
// {
//     /* ��������֪ͨ�������������� */
//     xTaskNotifyFromISR(refereeSystemTaHandle, send_date, eSetValueWithOverwrite, NULL);
// }
