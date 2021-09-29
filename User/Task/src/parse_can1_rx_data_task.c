#include "parse_can1_rx_data_task.h"

extern osThreadId parseCan1RxDataHandle;
static const uint16_t can1_get_data_signal = 0x0001;
static const uint8_t can1_motor_device_number = 4;
static const uint8_t can1_rx_data_overtime = 100;
static Motor_Measure_t m3508_feddback_data[can1_motor_device_number];

void StartParseCan1RxDataTask(void const * argument)
{
    // debug_print("cansig\r\n");
    osEvent can1_get_data_event;

    Can1_Filter_Init();

    osDelay(100);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    // debug_print("cansig\r\n");
    for (;;)
    {
        osDelay(10);
        debug_print("cansig\r\n");
        can1_get_data_event = osSignalWait(can1_get_data_signal, can1_rx_data_overtime);
        // debug_print("cansig\r\n");
        if (can1_get_data_event.status == osEventSignal)
        {
            debug_print("cansignal\r\n");
            if (can1_get_data_event.value.signals == can1_get_data_signal)
            {
                debug_print("cansssss   ig\r\n");
                Parse_Can1_Rxd_Data();
                debug_showdata1("mechine angle", m3508_feddback_data[0].mechanical_angle);
                debug_showdata1("speed", m3508_feddback_data[0].speed_rpm);
                debug_showdata1("temperate", m3508_feddback_data[0].temperate);
                osDelay(10);
            }
        
        }
    }
}

/**
 * @brief           通知 CAN1 数据解析任务进行数据解析
 * @param[in]       void
 * @retval          void
 */
void Info_Can1_ParseData_Task(void)
{
    // debug_print("can1\r\n");
    osSignalSet(parseCan1RxDataHandle, can1_get_data_signal);
    Parse_Can1_Rxd_Data();
}

/**
 * @brief           解析 CAN1 上的反馈数据
 * @param[in]       void
 * @retval          void
 */
void Parse_Can1_Rxd_Data(void)
{
    CAN_RxHeaderTypeDef can1_rx_header;
    static uint8_t i = 0;
    static uint8_t can1_rx_data[8];
    // debug_print("can2\r\n");
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &can1_rx_header, can1_rx_data);
    // debug_print("can3\r\n");
    i = can1_rx_header.StdId - CAN_3508_M1_ID;
    // debug_showdata1("id", i);
    GET_MOTOR_MEASURE_DATA(&(m3508_feddback_data[i]), can1_rx_data);
    // debug_showdata1("id", i+1);
    // debug_showdata1("mechine angle", m3508_feddback_data[0].mechanical_angle);
    // debug_showdata1("speed", m3508_feddback_data[0].speed_rpm);
    // debug_showdata1("temperate", m3508_feddback_data[0].temperate);
}

/**
 * @brief           返回 CAN1 总线上电机的反馈数据
 * @param[in]       void
 * @return          Motor_Measure_t* 反馈电机数据结构体指针
 */
Motor_Measure_t *Get_Can1_Feedback_Data(void)
{
    return m3508_feddback_data;
}

/**
 * @brief           返回 CAN1 总线上电机的数量
 * @param[in]       void
 * @return          const uint8_t* CAN1 总线上电机数量
 */
const uint8_t *Get_Can1_Motor_DeviceNumber(void)
{
    return (&can1_motor_device_number);
}
