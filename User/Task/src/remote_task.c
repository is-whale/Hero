#include "remote_task.h"

static Rc_Ctrl_t remote_controller;                    ///< 本次解析的遥控器数据
static Rc_Ctrl_t last_time_remote_controller;          ///< 上次解析的遥控器数据
static Robot_control_data_t robot_control_data;
static uint16_t *sbus_rxd_len;                         ///< 本次遥控器接收到的数据长度
static uint8_t *rc_rx_buffer[2];                       ///< 遥控器两次接收数据缓冲
static const uint32_t remote_get_data_signal = 0x0001; ///< 遥控器接收数据信号
static const uint32_t remote_data_overtime = 50;      ///< 遥控器接收数据包超时时间
extern osThreadId remoteTaskHandle;                    ///< 遥控器任务句柄

void StartRemoteTask(void const *argument)
{
    static uint8_t rx_available_buffer_index;
    osEvent remote_get_data_event;

    rc_rx_buffer[0] = Get_Usart1_DMA_RxBuffer_One();
    rc_rx_buffer[1] = Get_Usart1_DMA_RxBuffer_Two();
    sbus_rxd_len = Get_Usart1_DMA_Rxd_DataLen();

    Usart1_RxDMA_Init();

    Rc_Data_Reset(&remote_controller);
    Rc_Data_Reset(&last_time_remote_controller);

    osDelay(100);

    for (;;)
    {
        remote_get_data_event = osSignalWait(remote_get_data_signal, remote_data_overtime);
        if (remote_get_data_event.status == osEventSignal)
        {
            if (remote_get_data_event.value.signals == remote_get_data_signal)
            {
                rx_available_buffer_index = Get_Rc_Available_Bufferx();

                if (Rc_Data_Check_Parse(rc_rx_buffer[rx_available_buffer_index], &remote_controller, *sbus_rxd_len))
                {
                    Parse_Robot_Control_Data(&remote_controller, &last_time_remote_controller, &robot_control_data); ///< 解析出遥控器模式
                    
                    Rc_Data_Copy(&last_time_remote_controller, &remote_controller); ///< 保存本次遥控器状态

                    Module_Reload(remote_control); ///< 更新遥控器状态
                }
                else
                {
                    Usart1_Rx_DMA_Reset();
                    Rc_Data_Copy(&remote_controller, &last_time_remote_controller); ///< 校验出错保持遥控器数据不变
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
 * @brief           通知 remoteTask 进行遥控器数据的解析，在 USART1 的串口中断中调用
 * @param[in]       none
 * @retval          void
 */
void Info_RemoteTask_Parse_Data(void)
{
    osSignalSet(remoteTaskHandle, remote_get_data_signal);
}

/**
 * @brief                       获取解析后的遥控器数据
 * @param[in]                   void
 * @retval {Rc_Ctrl_t*}        解析后的遥控器数据结构体指针
 */
Rc_Ctrl_t *Get_Rc_Parsed_RemoteData_Pointer(void)
{
    return &remote_controller;
}
