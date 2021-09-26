
#include "usart3.h"

static uint8_t usart3_tx_lock = 0;
static char usart3_dma_transmit_buf[256];
static const uint16_t usart3_dma_rx_max_len = 50;
volatile uint8_t usart3_dma_rx_buffer[usart3_dma_rx_max_len];

/**
 * @description: ����3�Ķ����ʼ��?
 * @param {*}
 * @return {*}
 */
void Usart3_Tx_Init(void)
{
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_3, LL_USART_DMA_GetRegAddr(USART3));
	LL_USART_EnableDMAReq_TX(USART3);
}

/**
 * @description: ����3ʹ��DMA��ʽ��������
 * @param {uint32_t} data_address �������ݵĵ��׵�ַ
 * @param {uint32_t} len �������ݵĳ���
 * @return {*}
 */
void Usart3_Transmit_Dma(uint32_t data_address, uint32_t len)
{
	if(usart3_tx_lock == 0)
	{
		usart3_tx_lock = 1;
		LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_3, data_address);
		LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_3, len);
		LL_USART_ClearFlag_TC(USART3);
		LL_DMA_ClearFlag_TC3(DMA1);
		LL_USART_EnableIT_TC(USART3);
		LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_3);
	}
}

/**
 * @description: ��������3 TC�ж�ʱ����
 * @param {*}
 * @return {*}
 */
void Usart3_It_Tc_Callback(void)
{
	//TODO:����print����
	LL_USART_DisableIT_TC(USART3);
	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_3);
}


/**
 * @description	
 * @param {*}
 * @return {*}
 */
void Usart3_Rx_Init(void)
{
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_1, LL_USART_DMA_GetRegAddr(USART3));
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_1, (uint32_t)(usart3_dma_rx_buffer));

	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1, usart3_dma_rx_max_len);

	LL_USART_ClearFlag_IDLE(USART3);
	LL_USART_EnableIT_IDLE(USART3);

	LL_USART_EnableDMAReq_RX(USART3);

	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);
}

uint8_t *Get_Usart3_DMA_RxBuffer(void)
{
	return (uint8_t *)usart3_dma_rx_buffer;
}

const uint16_t *Get_Usart3_DMA_Rx_MaxLen(void)
{
	return &usart3_dma_rx_max_len;
	usart3_tx_lock = 0;
	LL_USART_DisableIT_TC(USART3);
	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_3);
}

int __printf(const char *format, ...)
{
	uint32_t len;
	va_list args;
	va_start(args, format);
	len = vsnprintf((char*)usart3_dma_transmit_buf,sizeof(usart3_dma_transmit_buf),(char*)format,args);
	va_end(args);
	Usart3_Transmit_Dma((uint32_t)usart3_dma_transmit_buf, len);
	return 1;
}
