#include "shell_task.h"

static uint8_t *shell_rx_buffer;
const uint16_t *shell_rx_max_buffer_len;
static uint16_t shell_rxd_data_len;
extern osSemaphoreId shellGetDataBinarySemHandle;

void StartShellTask(void const *argument)
{
    shell_rx_buffer = Get_Usart3_DMA_RxBuffer();
    shell_rx_max_buffer_len = Get_Usart3_DMA_Rx_MaxLen();

    PRINT_USART_INIT();

    osDelay(100);

    for (;;)
    {
        if (osOK == osSemaphoreWait(shellGetDataBinarySemHandle,osWaitForever))
        {
            // TODO
        }
        osDelay(10);
    }
}
static void Inform_ShellTask_Get_Data(void)
{
    osSemaphoreRelease(shellGetDataBinarySemHandle);
}

void Usart3_Idle_ITCallback(void)
{
    if (LL_USART_IsActiveFlag_IDLE(USART3))
    {
        ///< 关闭 DMA
        LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_1);

        ///< 清除 IDLE 中断
        LL_DMA_ClearFlag_TC1(DMA1);
        LL_USART_ClearFlag_IDLE(USART3);

        ///< 计算收到额数据的长度   
        shell_rxd_data_len = (*shell_rx_max_buffer_len) - LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_1);

        // printf("rx len:%d\r\n", shell_rxd_data_len);
        // printf("rx :%s\r\n", shell_rx_buffer);

        ///< 重新设置传输长度
        LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1, *shell_rx_max_buffer_len);

        ///< 通知任务进行解析
        Inform_ShellTask_Get_Data();
        
        ///< 重新开启 DMA
        LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);
    }
}

// int fputc(int ch, FILE *f)
// {
//     LL_USART_TransmitData8(USART3, ch);
//     while (!LL_USART_IsActiveFlag_TC(USART3))
//     {
//     }
//     return ch;
// }
