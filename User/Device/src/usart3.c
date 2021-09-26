#include "usart3.h"

/**
 * @description: 串口3的额外初始化
 * @param {*}
 * @return {*}
 */
void Usart3_Tx_Init(void)
{
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_3, LL_USART_DMA_GetRegAddr(USART3));
	LL_USART_EnableDMAReq_TX(USART3);
}

/**
 * @description: 串口3使用DMA方式发送数据
 * @param {uint32_t} data_address 发送数据的的首地址
 * @param {uint32_t} len 发送数据的长度
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
 * @description: 发生串口3 TC中断时调用
 * @param {*}
 * @return {*}
 */
void Usart3_It_Tc_Callback(void)
{
	//TODO:解锁print互斥
	LL_USART_DisableIT_TC(USART3);
	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_3);
}
