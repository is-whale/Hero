#ifndef __MATH2_H_
#define __MATH2_H_

#include "stm32f4xx.h"

void Int16_Constrain(int16_t *data, int16_t min_value, int16_t max_value);
int16_t Int16_Limit(int16_t data, int16_t min_value, int16_t max_value);

void Int16_Add_Limit(int16_t *data, int16_t add_value, int16_t max_value);
void Int16_Minus_Limit(int16_t *data, int16_t minus_value, int16_t min_value);
void Float_Constraion(float *data, float min_value, float max_value);

#endif // __MATH2_H_
