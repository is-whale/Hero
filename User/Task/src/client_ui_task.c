#include "client_ui_task.h"

void StartClientUiTask(void const *argument)
{
    osDelay(1000);
    Uart8_Dma_Tx_Init();
    uint8_t data[4] = {0x01, 0x02, 0x03, 0x04};
    for (;;)
    {
        // printf_ui("%d %d %d %d\r\n", 1, 2, 3, 4);
        Uart8_Transmit_Dma((uint32_t)data, 4);
        osDelay(50);
    }
}
