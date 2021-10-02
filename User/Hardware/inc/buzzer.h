#ifndef __BUZZER_H_
#define __BUZZER_H_

#include "tim.h"

#define BUZZER_ON(pulse)                           \
    {                                              \
        TIM12->ARR = pulse;                        \
        LL_TIM_OC_SetCompareCH1(TIM12, pulse / 2); \
    }
#define BUZZER_OFF() LL_TIM_OC_SetCompareCH1(TIM12, 0);

void Buzzer_Init(void);

#endif //__BUZZER_H_
