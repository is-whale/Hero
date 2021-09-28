#include "usart1.h"

static const uint16_t usart1_dma_rx_max_len = 36;					  ///< USART1 DMA �����ճ���
static volatile uint8_t usart1_dma_rx_buffer0[usart1_dma_rx_max_len]; ///< USART1 DMA ���ܻ����� 1
static volatile uint8_t usart1_dma_rx_buffer1[usart1_dma_rx_max_len]; ///< USART1 DMA ���ܻ����� 2
static volatile uint16_t usart1_dma_rxd_data_len;				      ///< USART1 DMA �Ѿ����յ������ݳ���

/**
 * @brief       ��ʼ������ 1 �Ľ��� DMA 
 * @param[in]   none
 * @retval      void
 */
void Usart1_RxDMA_Init(void)
{
	LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_2, LL_USART_DMA_GetRegAddr(USART1));
	LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_2, (uint32_t)(usart1_dma_rx_buffer0));
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_2, usart1_dma_rx_max_len);

	///< ���� DMA ˫����
	LL_DMA_SetMemory1Address(DMA2, LL_DMA_STREAM_2, (uint32_t)(usart1_dma_rx_buffer1));
	LL_DMA_SetCurrentTargetMem(DMA2, LL_DMA_STREAM_2, LL_DMA_CURRENTTARGETMEM0);
	LL_DMA_EnableDoubleBufferMode(DMA2, LL_DMA_STREAM_2);

	LL_USART_ClearFlag_IDLE(USART1);
	LL_USART_EnableIT_IDLE(USART1);

	LL_USART_EnableDMAReq_RX(USART1);

	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_2);
}

/**
 * @brief           ���� 1 �Ľ����жϺ��� 
 * @param[in]       none
 * @retval          void
 */
void Usart1_DMA_RxCp_Callback(void)
{
	if (LL_USART_IsActiveFlag_IDLE(USART1))
	{
		///< �ر� DMA
		LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_2);

		///< ��� IDLE �жϡ�DMA TC2 ��־λ
		LL_USART_ClearFlag_IDLE(USART1);
		LL_DMA_ClearFlag_TC2(DMA2);

		///< ��ȡ��֡�����ݳ���
		usart1_dma_rxd_data_len = usart1_dma_rx_max_len - LL_DMA_GetDataLength(DMA2, LL_DMA_STREAM_2);

		if (LL_DMA_GetCurrentTargetMem(DMA2, LL_DMA_STREAM_2) == LL_DMA_CURRENTTARGETMEM1)
		{
			LL_DMA_SetCurrentTargetMem(DMA2, LL_DMA_STREAM_2, LL_DMA_CURRENTTARGETMEM0);
		}
		else
		{
			LL_DMA_SetCurrentTargetMem(DMA2, LL_DMA_STREAM_2, LL_DMA_CURRENTTARGETMEM1);
		}

		///< �����������ݴ��䳤��
		LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_2, usart1_dma_rx_max_len);

		///< ֪ͨ������н���
		Info_RemoteTask_Parse_Data();

		///< ���¿��� DMA
		LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_2);
	}
}

/**
 * @brief           ���� USART1 DMA �ĵ�һ�����ܻ�����ָ��
 * @param[in]       none
 * @retval          USART1 DMA �Ľ��ܻ�������ַ
 */
uint8_t *Get_Usart1_DMA_RxBuffer_One(void)
{
	return (uint8_t *)usart1_dma_rx_buffer0;
}

/**
 * @brief           ���� USART1 DMA �Ľ��ܻ����������ܳ���
 * @param[in]       none
 * @retval          USART1 DMA �������ճ���
 */
const uint16_t *Get_Usart1_DMA_RxMaxLen(void)
{
	return &usart1_dma_rx_max_len;
}

/**
 * @brief           ���� USART1 DMA �Ľ��ܻ������Ѿ����ܵ������ݳ���
 * @param[in]       none
 * @retval          USART1 DMA �Ѿ����ܵ��˶೤������
 */
uint16_t *Get_Usart1_DMA_Rxd_DataLen(void)
{
	return (uint16_t*)&usart1_dma_rxd_data_len;
}

/**
 * @brief           ���� USART1 DMA �ڶ������ջ�����
 * @param[in]       none
 * @retval          USART1 DMA �Ľ��ܻ�������ַ
 */
uint8_t *Get_Usart1_DMA_RxBuffer_Two(void)
{
	return (uint8_t *)(usart1_dma_rx_buffer1);
}

/**
 * @brief           ���� USART1 DMA �ĵ�ǰ���õĽ��ܻ�����
 * @param[in]       none
 * @retval          USART1 DMA �ĵ�ǰ���õĽ��ܻ�����
 */
uint8_t Get_Rc_Available_Bufferx(void)
{
	if (LL_DMA_GetCurrentTargetMem(DMA2, LL_DMA_STREAM_2))
	{
		return 1;
	}
	return 0;
}

/**
 * @brief           ���� USART1 Rx DMA
 * @param[in]       none
 * @retval          void
 */
void Usart1_Rx_DMA_Reset(void)
{
	LL_USART_Disable(USART1);
	LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_2);
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_2, usart1_dma_rx_max_len);

	LL_USART_ClearFlag_IDLE(USART1);
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_2);
	LL_USART_Enable(USART1);
}
