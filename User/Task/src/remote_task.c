/**
 * @file remote_task.c
 * @brief ң�������ݽ������񣬽�����ң�������ݣ�������ң�����������û����˵�ǰģʽ���õ���ز���
 * @version 0.1
 * @date 2021-9-27
 * @copyright Copyright (c) 2021
 */

#include "remote_task.h"

static Rc_Ctrl_t remote_controller;                    ///< ���ν�����ң��������
static Rc_Ctrl_t last_time_remote_controller;          ///< �ϴν�����ң��������
static Robot_control_data_t robot_control_data;        ///< ������ģʽ��������
static uint16_t *sbus_rxd_len;                         ///< ����ң�������յ������ݳ���
static uint8_t *rc_rx_buffer[2];                       ///< ң�������ν������ݻ���
static const uint32_t remote_get_data_signal = 0x0001; ///< ң�������������ź�
static const uint32_t remote_data_overtime = 50;       ///< ң�����������ݰ���ʱʱ��
extern osThreadId remoteTaskHandle;                    ///< ң����������

void StartRemoteTask(void const *argument)
{
    static uint8_t rx_available_buffer_index;
    osEvent remote_get_data_event;

    rc_rx_buffer[0] = Get_Usart1_DMA_RxBuffer_One();
    rc_rx_buffer[1] = Get_Usart1_DMA_RxBuffer_Two();
    sbus_rxd_len = Get_Usart1_DMA_Rxd_DataLen();

    Usart1_RxDMA_Init();
    Rc_Data_Reset(&remote_controller);           ///< ������������
    Rc_Data_Reset(&last_time_remote_controller); ///< �ϴ���������
    Control_Data_Init(&robot_control_data);      ///< ��ʼ��ʱ�Ļ�����ģʽ����

    osDelay(100);

    for (;;)
    {
        remote_get_data_event = osSignalWait(remote_get_data_signal, remote_data_overtime);
        //���ó�ʱʱ������ͬ��������
        if (remote_get_data_event.status == osEventSignal)
        {
            if (remote_get_data_event.value.signals == remote_get_data_signal)
            {
                rx_available_buffer_index = Get_Rc_Available_Bufferx();

                if (Rc_Data_Check_Parse(rc_rx_buffer[rx_available_buffer_index], &remote_controller, *sbus_rxd_len)) ///<�������ݲ�����
                {
                    Parse_Robot_Control_Data(&remote_controller, &last_time_remote_controller, &robot_control_data); ///< ������ң����ģʽ

                    Rc_Data_Copy(&last_time_remote_controller, &remote_controller); ///< ���汾��ң����״̬

                    Module_Reload(remote_control); ///< ����ң����״̬���������ж�
                }
                else
                {
                    Usart1_Rx_DMA_Reset();                                          ///< DMA����
                    Rc_Data_Copy(&remote_controller, &last_time_remote_controller); ///< У�������ң�������ݲ���
                }
            }
        }
        else if (remote_get_data_event.status == osEventTimeout)
        {
            Rc_Data_Reset(&remote_controller);

            Rc_Data_Reset(&last_time_remote_controller);
        }
    }
}

/**
 * @brief           ֪ͨ remoteTask ����ң�������ݵĽ������� USART1 �Ĵ����ж��е���
 * @param[in]       none
 * @retval          void
 */

void Info_RemoteTask_Parse_Data(void)
{
    osSignalSet(remoteTaskHandle, remote_get_data_signal);
}

/**
 * @brief                       ��ȡ�������ң��������
 * @param[in]                   void
 * @retval {Rc_Ctrl_t*}         �������ң�������ݽṹ��ָ��
 */
Rc_Ctrl_t *Get_Rc_Parsed_RemoteData_Pointer(void)
{
    return &remote_controller;
}

/**
 * @brief                       ��ȡ������Ļ�����ģʽ�ṹ��ָ��
 * @param[in]                   void
 * @retval {Rc_Ctrl_t*}         ������Ľ�����Ļ�����ģʽ�ṹ��ָ��
 */
Robot_control_data_t *Get_Parsed_RobotMode_Pointer(void)
{
    return &robot_control_data;
}
