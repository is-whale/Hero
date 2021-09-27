#include "usart1.h"

static const uint16_t usart1_dma_rx_max_len = 128;
volatile uint8_t usart1_dma_rx_buffer[usart1_dma_rx_max_len];
volatile uint8_t usart1_dma_rx_buffer_second[usart1_dma_rx_max_len];
static uint16_t usart1_dma_rxd_data_len;

/**
 * @brief       初始化串口 1 的接收 DMA 
 * @param[in]   none
 * @retval      void
 */
void Usart1_RxDMA_Init(void)
{
    LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_2, LL_USART_DMA_GetRegAddr(USART1));
    LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_2, (uint32_t)(usart1_dma_rx_buffer));
    LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_2, usart1_dma_rx_max_len);

    ///< 开启 DMA 双缓冲
    LL_DMA_SetMemory1Address(DMA2, LL_DMA_STREAM_2, (uint32_t)(usart1_dma_rx_buffer_second));
    LL_DMA_SetCurrentTargetMem(DMA2, LL_DMA_STREAM_2, LL_DMA_CURRENTTARGETMEM0);
    LL_DMA_EnableDoubleBufferMode(DMA2, LL_DMA_STREAM_2);

    LL_USART_ClearFlag_IDLE(USART1);
    LL_USART_EnableIT_IDLE(USART1);

    LL_USART_EnableDMAReq_RX(USART1);

    LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_2);
}

/**
 * @brief           串口 1 的接收中断函数 
 * @param[in]       none
 * @retval          void
 */
void Usart1_DMA_RxCp_Callback(void)
{
    if (LL_USART_IsActiveFlag_IDLE(USART1))
    {
        ///< 关闭 DMA
        LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_2);

        ///< 清楚 IDLE 中断、DMA TC2 标志位
        LL_USART_ClearFlag_IDLE(USART1);
        LL_DMA_ClearFlag_TC2(DMA2);

        ///< 获取该帧的数据长度
        usart1_dma_rxd_data_len = usart1_dma_rx_max_len - LL_DMA_GetDataLength(DMA2, LL_DMA_STREAM_2);
    
        // debug_print("rx len:%d\r\n",usart1_dma_rxd_data_len);
        ///< 重新设置数据传输长度
        LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_2, usart1_dma_rx_max_len);

        ///< 通知任务进行解析
        Info_RemoteTask_Parse_Data();

        ///< 重新开启 DMA
        LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_2);
    }
}

uint8_t *Get_Usart1_DMA_RxBuffer_One(void)
{
    return (uint8_t *)usart1_dma_rx_buffer;
}

const uint16_t *Get_Usart1_DMA_RxMaxLen(void)
{
    return &usart1_dma_rx_max_len;
}

uint16_t *Get_Usart1_DMA_Rxd_DataLen(void)
{
    return &usart1_dma_rxd_data_len;
}
uint8_t *Get_Usart1_DMA_RxBuffer_Two(void)
{
    return (uint8_t *)(usart1_dma_rx_buffer_second);
}

uint8_t Get_Rc_Available_Bufferx(void)
{
    if (LL_DMA_GetCurrentTargetMem(DMA2, LL_DMA_STREAM_2))
    {
        return 1;
    }
    return 0;
}
