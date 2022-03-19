#include "shell_task.h"

static uint8_t *shell_rx_buffer;
const uint16_t *shell_rx_max_buffer_len;
static uint16_t shell_rxd_data_len;
extern osSemaphoreId shellGetDataBinarySemHandle;
extern osThreadId shellTaskHandle;
static const uint32_t shell_get_data_signal = 0x0001;

static void Shell_Str_Process(void);
static void Print_At_First(void);

void StartShellTask(void const *argument)
{
	osEvent shell_get_data_event;
	shell_rx_buffer = Get_Usart3_DMA_RxBuffer();
	shell_rx_max_buffer_len = Get_Usart3_DMA_Rx_MaxLen();
	PRINT_USART_INIT();
	Shell_Init();
	osDelay(25);
	Print_At_First();
	osDelay(25);

	for (;;)
	{
		shell_get_data_event = osSignalWait(shell_get_data_signal, osWaitForever);
		if (shell_get_data_event.status == osEventSignal)
		{
			if (shell_get_data_event.value.signals == shell_get_data_signal)
			{
				///< �ַ���Ԥ����
				Shell_Str_Process();
				Shell_Command_Parse((char *)shell_rx_buffer);
			}
		}
	}
}

/**
 * @brief 			֪ͨ shell ����Խ��յ������ݽ��н������ڴ��� 3 �� idle �ж��е���
 * @param[in]		void
 * @retval			void
 */
static void Inform_ShellTask_Get_Data(void)
{
	osSignalSet(shellTaskHandle, shell_get_data_signal);
}

/**
 * @brief 				���� 3 �� idle �жϴ�������������Ӧ�÷�������淶��Ӧ�÷��� usart3.c �ļ���
 * @param[in]			void
 * @retval				void
 */
void Usart3_Idle_ITCallback(void)
{
	if (LL_USART_IsActiveFlag_IDLE(USART3))
	{
		///< �ر� DMA
		LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_1);

		///< ��� IDLE �ж�
		LL_DMA_ClearFlag_TC1(DMA1);
		LL_USART_ClearFlag_IDLE(USART3);

		///< �����յ������ݵĳ���
		shell_rxd_data_len = (*shell_rx_max_buffer_len) - LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_1);

		///< �������ô��䳤��
		LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1, *shell_rx_max_buffer_len);

		///< ֪ͨ������н���
		Inform_ShellTask_Get_Data();

		///< ���¿��� DMA
		LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);
	}
}

/**
 * @brief 				��ӡ logo : dragon
 * @param[in]			void
 * @retval				void
 */
void Print_Dragon(void)
{
	__printf("     /\\_____/\\     \r\n");
	__printf("    /  o   o  \\    \r\n");
	__printf("   ( ==  ^  == )   \r\n");
	__printf("    )         (    \r\n");
	__printf("   (           )   \r\n");
	__printf("  ( (  )   (  ) )  \r\n");
	__printf(" (__(__)___(__)__) \r\n");
}

/**
 * @brief 				��ӡ logo : hll super hero
 * @param[in]			void
 * @retval				void
 */
void Print_Logo(void)
{
	__printf("    __  ____    __       _____                          __  __              \r\n");
	__printf("   / / / / /   / /      / ___/__  ______  ___  _____   / / / /__  _________ \r\n");
	__printf("  / /_/ / /   / /       \\__ \\/ / / / __ \\/ _ \\/ ___/  / /_/ / _ \\/ ___/ __ \r\n");
	__printf(" / __  / /___/ /___    ___/ / /_/ / /_/ /  __/ /     / __  /  __/ /  / /_/ /\r\n");
	__printf("/_/ /_/_____/_____/   /____/\\__,_/ .___/\\___/_/     /_/ /_/\\___/_/   \\____/ \r\n");
	__printf("                                /_/                                         \r\n");
}

/**
 * @brief 				������������ 3 ���յ������ݣ�ȥ������
 * @param[in]			void
 * @retval				void
 */
static void Shell_Str_Process(void)
{
	if (shell_rx_buffer[shell_rxd_data_len - 2] == '\r' && shell_rx_buffer[shell_rxd_data_len - 1] == '\n')
	{
		shell_rx_buffer[shell_rxd_data_len - 2] = '\0';
	}
	else
	{
		shell_rx_buffer[shell_rxd_data_len] = '\0';
	}
}

/**
 * @brief 				��ӡ AT ָ��
 * @param[in]			void
 * @retval				void
 */
static void Print_At_First(void)
{
	__printf("\r\n");
	// Print_Dragon();
	__printf("\r\n");
	Print_Logo();
	__printf("\r\n");
	__printf("Hll super hero 2022.\r\n");
	__printf("\r\n");
	__printf("\r\n");
}
