#include "externel_gyroscope_task.h"
#include "usart6.h"

extern osThreadId referenceGyroscHandle;
static const int32_t gyroscope_get_data_signal = 0x00110011;
static uint8_t usart6_rx_available_buffer_index;
static const uint8_t wt631_package_len = 33;
static uint16_t *externel_gyroscope_rxd_len;
static uint8_t *externel_gyroscope_rxd_buffer[2]; /* �����ǽ��ջ�����ָ������ */
static Wt61c_Data_t wt61c_data;                   /* ���������ݽṹ�� */

void StartextErnalGyroscopeParseTask(void const *argument)
{
    osEvent gyroscope_get_data_event; /* �����ǽ��������¼� */

    Usart6_RxDMA_Init();

    externel_gyroscope_rxd_len = Get_Usart6_DMA_Rxd_DataLen();
    externel_gyroscope_rxd_buffer[0] = Get_Usart6_DMA_RxBuffer_One();
    externel_gyroscope_rxd_buffer[1] = Get_Usart6_DMA_RxBuffer_Two();

    osDelay(1000);

    for (;;)
    {
        gyroscope_get_data_event = osSignalWait(gyroscope_get_data_signal, osWaitForever);
        if (gyroscope_get_data_event.status == osEventSignal)
        {
            if (gyroscope_get_data_event.value.signals == gyroscope_get_data_signal)
            {
                usart6_rx_available_buffer_index = Get_Usart6_Available_Bufferx();
                if (Check_Wt61c_Data_Available(
                        externel_gyroscope_rxd_buffer[usart6_rx_available_buffer_index],
                        *externel_gyroscope_rxd_len))
                {
                    Module_Reload(gyroscope);
                    switch (externel_gyroscope_rxd_buffer[usart6_rx_available_buffer_index][23])
                    {
                    //��������Э�飬�����Ƕ����ݰ�
                    case 0x53:
                        wt61c_data.angle.roll_x =
                            ((short)(externel_gyroscope_rxd_buffer[usart6_rx_available_buffer_index][25] << 8 |
                                     externel_gyroscope_rxd_buffer[usart6_rx_available_buffer_index][24])) /
                            32768.0 * 180;
                        wt61c_data.angle.pitch_y = ((short)(externel_gyroscope_rxd_buffer[usart6_rx_available_buffer_index][27] << 8 |
                                                            externel_gyroscope_rxd_buffer[usart6_rx_available_buffer_index][26])) /
                                                   32768.0 * 180;
                        wt61c_data.angle.yaw_z = ((short)(externel_gyroscope_rxd_buffer[usart6_rx_available_buffer_index][29] << 8 |
                                                          externel_gyroscope_rxd_buffer[usart6_rx_available_buffer_index][28])) /
                                                 32768.0 * 180;
                        if (wt61c_data.angle.yaw_z < 0)
                            wt61c_data.angle.yaw_z += 360;
                        break;
                    }
                }
            }
        }
        else if (gyroscope_get_data_event.status == osEventTimeout)
        {
            //! δ��ʱ���յ�����ϵͳ���ݣ�������Ӧ�Ĵ���
            Usart6_Rx_DMA_Reset();
        }
        osDelay(10);
    }
}

/**
* @brief    ֪ͨ�����ǽ���task��������
 */
void Inform_Gyroscope_Task_Parse_Data(void)
{
    osSignalSet(referenceGyroscHandle, gyroscope_get_data_signal);
}

/**
* @brief    ��������������Ƿ���ȷ
 */
uint8_t Check_Wt61c_Data_Available(const uint8_t *wt61c_raw_buf, uint16_t length)
{
    uint8_t sum_add = 0;
    if (length != wt631_package_len)
    {
        return 0;
    }
    if (wt61c_raw_buf[0] != 0x55)
    {
        return 0;
    }
    for (uint8_t i = 22; i < 32; i++)
    {
        sum_add += wt61c_raw_buf[i];
    }
    if (wt61c_raw_buf[32] != sum_add)
    {
        return 0;
    }
    return 1;
}
/**
* @brief    �������������ݵ�ַ
 */
Wt61c_Data_t *Get_Gyroscope_Data_t(void)
{
    return &wt61c_data;
}
