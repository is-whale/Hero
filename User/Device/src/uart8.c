#include "uart8.h"

static const uint16_t uart8_dma_rx_max_len = 256;                   //! UART8 DMA �����ճ���
static volatile uint8_t uart8_dma_rx_buffer0[uart8_dma_rx_max_len]; //! UART8 DMA ���ܻ����� 1
static volatile uint8_t uart8_dma_rx_buffer1[uart8_dma_rx_max_len]; //! UART8 DMA ���ܻ����� 2
static volatile uint16_t uart8_dma_rxd_data_len;                    //! UART8 DMA �Ѿ����յ������ݳ���

static volatile char uart8_dma_transmit_buf[256];

void Uart8_Dma_Tx_Init(void)
{
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_0, LL_USART_DMA_GetRegAddr(UART8));
	LL_USART_EnableDMAReq_TX(UART8);
}

void Uart8_Transmit_Dma(uint32_t data_address, uint32_t len)
{
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_0, data_address);
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_0, len);
	LL_USART_ClearFlag_TC(UART8);
	LL_DMA_ClearFlag_TC0(DMA1);
	LL_USART_EnableIT_TC(UART8);
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_0);
}

void Uart8_It_Tc_Callback(void)
{
	LL_USART_DisableIT_TC(UART8);
	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_0);
}

int printf_ui(const char *format, ...)
{
	uint32_t len;
	va_list args;
	va_start(args, format);
	len = vsnprintf((char *)uart8_dma_transmit_buf, sizeof(uart8_dma_transmit_buf), (const char *)format, args);
	va_end(args);
	Uart8_Transmit_Dma((uint32_t)uart8_dma_transmit_buf, len);
	while (!((UART8->SR) & USART_SR_TC))
	{

	}
	///< �ر� DMA
	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_0);
	return -1;
}

void Uart8_Rx_Dma_Init(void)
{
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_6, LL_USART_DMA_GetRegAddr(UART8));   //! ��������ĵ�ַ
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_6, (uint32_t)(uart8_dma_rx_buffer0)); //! �����ڴ�������ݵĵ�ַ
    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_6, uart8_dma_rx_max_len);                //! ���ý������ݵĳ���

    LL_DMA_SetMemory1Address(DMA1, LL_DMA_STREAM_6, (uint32_t)(uart8_dma_rx_buffer1)); //! ���� DMA ˫����
    LL_DMA_SetCurrentTargetMem(DMA1, LL_DMA_STREAM_6, LL_DMA_CURRENTTARGETMEM0);
    LL_DMA_EnableDoubleBufferMode(DMA1, LL_DMA_STREAM_6);

    LL_USART_ClearFlag_IDLE(UART8);
    LL_USART_EnableIT_IDLE(UART8);

    LL_USART_EnableDMAReq_RX(UART8);

    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_6);
}

void UART8_DMA_RxCp_Callback(void)
{
    if (LL_USART_IsActiveFlag_IDLE(UART8))
    {
        //! �ر� DMA
        LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_6);

        //! ��� IDLE �жϡ�DMA TC2 ��־λ
        LL_USART_ClearFlag_IDLE(UART8);
        LL_DMA_ClearFlag_TC2(DMA1);

        //! ��ȡ��֡�����ݳ���
        uart8_dma_rxd_data_len = uart8_dma_rx_max_len - LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_6);

        if (LL_DMA_GetCurrentTargetMem(DMA1, LL_DMA_STREAM_6) == LL_DMA_CURRENTTARGETMEM1)
        {
            LL_DMA_SetCurrentTargetMem(DMA1, LL_DMA_STREAM_6, LL_DMA_CURRENTTARGETMEM0);
        }
        else
        {
            LL_DMA_SetCurrentTargetMem(DMA1, LL_DMA_STREAM_6, LL_DMA_CURRENTTARGETMEM1);
        }

        //! �����������ݴ��䳤��
        LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_6, uart8_dma_rx_max_len);

        //! ֪ͨ������н���
        Inform_Reference_Analysis_Task_Pasre_Data();

        //! ���¿��� DMA
        LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_6);
    }
}

uint8_t Get_Reference_Available_Bufferx(void)
{
    if (LL_DMA_GetCurrentTargetMem(DMA1, LL_DMA_STREAM_6))
    {
        return 1;
    }
    return 0;
}

void Uart8_Rx_DMA_Reset(void)
{
    LL_USART_Disable(UART8);
    LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_6);
    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_6, uart8_dma_rx_max_len);

    LL_USART_ClearFlag_IDLE(UART8);
    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_6);
    LL_USART_Enable(UART8);
}

uint16_t *Get_Uart8_DMA_Rxd_DataLen(void)
{
    return (uint16_t *)&uart8_dma_rxd_data_len;
}

uint8_t *Get_Uart8_DMA_RxBuffer_Two(void)
{
    return (uint8_t *)(uart8_dma_rx_buffer1);
}
uint8_t *Get_Uart8_DMA_RxBuffer_One(void)
{
    return (uint8_t *)(uart8_dma_rx_buffer0);
}
