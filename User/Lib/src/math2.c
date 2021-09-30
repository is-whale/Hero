#include "math2.h"

/**
 * @brief int16���ͱ�����С����
 * @param {int16_t*} �����Ƶı���ָ�룬��ֱ�Ӹ��ĵ�ַ����ŵ�����
 * @param {int16_t} min_value ��Сֵ
 * @param {int16_t} max_value ���ֵ
 * @return {*}
 */
void Int16_Constrain(int16_t* data, int16_t min_value, int16_t max_value)
{
	if(*data > max_value) *data = max_value;
	else if(*data < min_value) *data = min_value;
}

/**
 * @brief int16���ͱ�����С����
 * @param {int16_t} data �����Ƶı���
 * @param {int16_t} min_value ��Сֵ
 * @param {int16_t} max_value ���ֵ
 * @return {int16_t} ���ƺ������
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
