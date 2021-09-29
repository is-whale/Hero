#include "parse_can1_rx_data_task.h"

extern osThreadId parseCan1RxDataHandle;
static const uint16_t can1_get_data_signal = 0x0001;
static const uint8_t can1_motor_device_number = 4;
static const uint8_t can1_rx_data_overtime = 100;
static Motor_Measure_t m3508_feddback_data[can1_motor_device_number];
static uint8_t *can1_rxd_data;
static CAN_RxHeaderTypeDef *can1_rx_header;

void StartParseCan1RxDataTask(void const *argument)
{
    osEvent can1_get_data_event;

    Can1_Filter_Init();
    can1_rxd_data = Get_CAN1_Rxd_Data();
    can1_rx_header = Get_CAN1_Rx_Header();

    osDelay(1000);

    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    for (;;)
    {
        can1_get_data_event = osSignalWait(can1_get_data_signal, can1_rx_data_overtime);
        if (can1_get_data_event.status == osEventSignal)
        {
            if (can1_get_data_event.value.signals == can1_get_data_signal)
            {
                Parse_Can1_Rxd_Data();
            }
        }
    }
}

/**
 * @brief           ֪ͨ CAN1 ���ݽ�������������ݽ���
 * @param[in]       void
 * @retval          void
 */
void Info_Can1_ParseData_Task(void)
{
    osSignalSet(parseCan1RxDataHandle, can1_get_data_signal);
}

/**
 * @brief           ���� CAN1 �ϵķ�������
 * @param[in]       void
 * @retval          void
 */
void Parse_Can1_Rxd_Data(void)
{
    static uint8_t i = 0;
    i = can1_rx_header->StdId - CAN_3508_M1_ID;
    Calculate_Motor_Data(&m3508_feddback_data[i], can1_rxd_data);
}

/**
 * @brief                       ���� CAN1 �����ϵ���ķ�������
 * @param[in]                   void
 * @return {Motor_Measure_t*}   ����������ݽṹ��ָ��
 */
Motor_Measure_t *Get_Can1_Feedback_Data(void)
{
    return m3508_feddback_data;
}

/**
 * @brief                     ���� CAN1 �����ϵ��������
 * @param[in]                 void
 * @return {const uint8_t*}   CAN1 �����ϵ������
 */
const uint8_t *Get_Can1_Motor_DeviceNumber(void)
{
    return (&can1_motor_device_number);
}
