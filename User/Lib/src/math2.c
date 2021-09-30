#include "math2.h"

/**
 * @brief int16类型变量大小限制
 * @param {int16_t*} 待限制的变量指针，会直接更改地址所存放的数据
 * @param {int16_t} min_value 最小值
 * @param {int16_t} max_value 最大值
 * @return {*}
 */
void Int16_Constrain(int16_t* data, int16_t min_value, int16_t max_value)
{
	if(*data > max_value) *data = max_value;
	else if(*data < min_value) *data = min_value;
}

/**
 * @brief int16类型变量大小限制
 * @param {int16_t} data 待限制的变量
 * @param {int16_t} min_value 最小值
 * @param {int16_t} max_value 最大值
 * @return {int16_t} 限制后的数据
 */
int16_t Int16_Limit(int16_t data, int16_t min_value, int16_t max_value)
{
	if(data > max_value) return max_value;
	else if(data < min_value) return min_value;
	return data;
}


void Int16_Add_Limit(int16_t* data, int16_t add_value,int16_t max_value)
{
	*data += add_value;
	if(*data > max_value)
	{
		*data = max_value;
	}
}

void Int16_Minus_Limit(int16_t* data, int16_t minus_value,int16_t min_value)
{
	*data -= minus_value;
	if(*data < min_value)
	{
		*data = min_value;
	}
}
