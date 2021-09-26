#include "usart3.h"

/**
 * @description: ����3�Ķ����ʼ��
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
void Usart3_Tx_Transmit_Dma(uint32_t data_address, uint32_t len)
{
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_3, data_address);
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_3, len);
	LL_USART_ClearFlag_TC(USART3);
	LL_DMA_ClearFlag_TC3(DMA1);
	LL_USART_EnableIT_TC(USART3);
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_3);
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
