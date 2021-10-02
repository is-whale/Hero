#include "buzzer.h"


void Buzzer_Init(void)
{
    LL_TIM_CC_EnableChannel(TIM12,LL_TIM_CHANNEL_CH1);
    LL_TIM_EnableAllOutputs(TIM12);
    LL_TIM_EnableCounter(TIM12);
}
