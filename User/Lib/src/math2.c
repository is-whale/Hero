#include "math2.h"

/**
 * @brief int16���ͱ�����С����
 * @param {int16_t*} �����Ƶı���ָ�룬��ֱ�Ӹ��ĵ�ַ����ŵ�����
 * @param {int16_t} min_value ��Сֵ
 * @param {int16_t} max_value ���ֵ
 * @return {*}
 */
void Int16_Constrain(int16_t *data, int16_t min_value, int16_t max_value)
{
	if (*data > max_value)
		*data = max_value;
	else if (*data < min_value)
		*data = min_value;
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
	if (data > max_value)
		return max_value;
	else if (data < min_value)
		return min_value;
	return data;
}

void Int16_Add_Limit(int16_t *data, int16_t add_value, int16_t max_value)
{
	*data += add_value;
	if (*data > max_value)
	{
		*data = max_value;
	}
}

void Int16_Minus_Limit(int16_t *data, int16_t minus_value, int16_t min_value)
{
	*data -= minus_value;
	if (*data < min_value)
	{
		*data = min_value;
	}
}

void Float_Constraion(float *data, float min_value, float max_value)
{
	if (*data > max_value)
	{
		*data = max_value;
	}
	else if (*data < min_value)
	{
		*data = min_value;
	}
}

/* �Ƕ�Pidʱ���ڻ�ȡtar��cur֮������ŵ���  0~8191 */
void Handle_Angle8191_PID_Over_Zero(float *tar, float *cur)
{
	if(*tar - *cur > 4096)   //4096 ����Ȧ��е�Ƕ�, ����趨ֵ��Ŀ��ֵ֮��ĽǶȲ��Ѿ������˰�Ȧ
	{
		*cur += 8192;
	}
	else if(*tar - *cur < -4096)
	{
		*cur = *cur - 8192;
	}
}

/**
  * @brief  fast inverse square-root, to calculate 1/Sqrt(x)
  * @param  x: the number need to be calculated
  * @retval 1/Sqrt(x)
  */
float Inv_Sqrt(float x) 
{
	float halfx = 0.5f * x;
	float y     = x;
	long  i     = *(long*)&y;
	
	i = 0x5f3759df - (i >> 1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	
	return y;
}