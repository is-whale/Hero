#include "shell_task.h"

static uint8_t *shell_rx_buffer;
const uint16_t *shell_rx_max_buffer_len;
static uint16_t shell_rxd_data_len;

void StartShellTask(void const *argument)
{
    shell_rx_buffer = Get_Usart3_DMA_RxBuffer();
    shell_rx_max_buffer_len = Get_Usart3_DMA_Rx_MaxLen();

    PRINT_USART_INIT();

    osDelay(100);

    for (;;)
    {
        osDelay(10);
    }
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

        printf("rx len:%d\r\n", shell_rxd_data_len);
        printf("rx :%s\r\n", shell_rx_buffer);

        ///< 重新设置传输长度
        LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1, *shell_rx_max_buffer_len);

        ///< 通知任务进行解析

        
        ///< 重新开启 DMA
        LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);
    }
}

void Usart_Send_Byte(USART_TypeDef *pUsart, uint8_t ch)
{
    LL_USART_TransmitData8(pUsart, ch);
    while (LL_USART_IsActiveFlag_TXE(pUsart) == RESET)
        ;
}

void Usart_Send_ArrayBuffer(USART_TypeDef *pUsart, uint8_t *array, uint16_t length)
{
    uint16_t i;
    for (i = 0; i < length; i++)
    {
        Usart_Send_Byte(pUsart, array[i]);
    }
}

void Usart_Send_String(USART_TypeDef *pUsart, char *str)
{
    unsigned int k = 0;
    do
    {
        Usart_Send_Byte(pUsart, *(str + k));
        k++;
    } while (*(str + k) != '\0');

    ///< 等待发送完成
    while (LL_USART_IsActiveFlag_TC(pUsart) == RESET)
        ;
}

void Usart_Send_HalfWord(USART_TypeDef *pUsart, uint16_t ch)
{
    uint8_t temp_h, temp_l;

    ///< 取出高八位
    temp_h = (ch & 0XFF00) >> 8;
    ///< 取出低八位
    temp_l = ch & 0XFF;

    ///< 发送高八位
    LL_USART_TransmitData8(pUsart, temp_h);
    while (LL_USART_IsActiveFlag_TXE(pUsart) == RESET)
        ;

    ///< 发送低八位
    LL_USART_TransmitData8(pUsart, temp_l);
    while (LL_USART_IsActiveFlag_TXE(pUsart) == RESET)
        ;
}

int fputc(int ch, FILE *f)
{
    LL_USART_TransmitData8(USART3, ch);
    while (!LL_USART_IsActiveFlag_TC(USART3))
    {
    }
    return ch;
}
