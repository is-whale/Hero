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
/**
 * @brief			float���������޷�
 * @param [in]		����ֵ�������޷�
 * @return			void
*/
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

/* �Ƕ�Pidʱ���ڻ�ȡtar��cur֮������ŵ���

��̨�Ƕȹ��㴦��	0~8191
 */
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

/* �Ƕ�Pidʱ���ڻ�ȡtar��cur֮������ŵ���

*��̨�Ƕȹ��㴦����ֹ��ת	0~360
*/
void Handle_Angle360_PID_Over_Zero(float *tar, float *cur)
{
	if(*tar - *cur > 180)   //4096 ����Ȧ��е�Ƕ�
	{
		*cur += 360;
	}
	else if(*tar - *cur < -180)
	{
		*cur = *cur - 360;
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

//���������float
float Hex4_To_Float1(unsigned char *array)
{
	unsigned char array_copy[4];
	float f;

	for(unsigned int i=0; i<4; i++)
	{
		array_copy[i]=array[i];
	}
	f = *((float *)((array_copy)));
	return f;
}
/**
 * @brief					��������ת��(���ڲ���ϵͳ2�ֽ����ݽ���)
 * @param [in] {uint8_t}
 * @return	{uint16_t}
 *
*/
uint16_t Uint8_t_Array_To_Uint16_t(uint8_t *array)
{
	return ( (array[1]<<8) | array[0] );
}

/**
  * @brief          б��������ʼ��
  * @author         RM
  * @param[in]      б�������ṹ��
  * @param[in]      �����ʱ�䣬��λ s
  * @param[in]      ���ֵ
  * @param[in]      ��Сֵ
  * @retval         ���ؿ�
  */
void ramp_init(ramp_function_source_t *ramp_source_type, float step,float limit)
{
    ramp_source_type->step = step;
    ramp_source_type->max_value = limit;
    ramp_source_type->min_value = -limit;
    ramp_source_type->input = 0.0f;
    ramp_source_type->out = 0.0f;
}

/**
  * @brief          б���������㣬���������ֵ���е��ӣ� ���뵥λΪ /s ��һ������������ֵ
  * @author         RM
  * @param[in]      б�������ṹ��
  * @param[in]      ����ֵ
  * @param[in]      �˲�����
  * @retval         ���ؿ�
  */
void ramp_calc(ramp_function_source_t *ramp_source_type, float input)
{
    ramp_source_type->input = input;
    ramp_source_type->out += ramp_source_type->input * ramp_source_type->step;
    if (ramp_source_type->out > ramp_source_type->max_value)
    {
        ramp_source_type->out = ramp_source_type->max_value;
    }
    else if (ramp_source_type->out < ramp_source_type->min_value)
    {
        ramp_source_type->out = ramp_source_type->min_value;
    }
}
