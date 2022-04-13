#include "referee_system.h"
#include "console_print.h"
#include "crc_check.h"
#include "math2.h"
#include "monitor_task.h"
#include "usart3.h"

/* ����ϵͳ���Ժ궨�� */
#if 1
#define DEBUG_LOG debug_log
#define DEBUG_ERROR debug_error
#else
#define DEBUG_LOG(format, arg) /* DEBUG OFF */
#define DEBUG_ERROR(err)	   /* DEBUG OFF */
#endif

#if 0
#define JUDGE_ARRAY DEBUG_ARRAY
#else
#define JUDGE_ARRAY(name, data, len)
#endif

#define DATA_LEN_CHECK(datalen, correct_len, err) \
	if (data_len != correct_len)                  \
	{                                             \
		DEBUG_ERROR(err);                         \
		return 0;                                 \
	}

//��������
void Find_All_A5(uint8_t *get_data, uint16_t data_len, uint8_t *r_position, uint8_t *r_a5_length);
uint8_t Analysis_Frame_Header(uint8_t *get_data, uint16_t *r_data_length, uint8_t *r_seq);
uint8_t Check_Package_Crc16(uint8_t *get_data, uint16_t data_len);
uint16_t Analysis_Cmd_Id(uint8_t *get_data);
uint8_t Analysis_Data(uint8_t *get_data, uint16_t data_len);

static uint8_t Analysis_Power_Heat_Data(uint8_t *data_package, uint16_t data_len);
static uint8_t Analysis_Shoot_Data(uint8_t *data_package, uint16_t data_len);
static uint8_t Analysis_Game_Robot_Status(uint8_t *data_package, uint16_t data_len);

//����ϵͳ���ݶ���
static Judge_data_t judge_data;

///<���ز���ϵͳ���ݽṹ���ַ
const Judge_data_t *Get_Referee_Data(void)
{
	return &judge_data;
}

/*
  ��������Analysis_Judge_System
  ����  ����������ϵͳ����
  ����  ��get_data��Ҫ������֡ͷ���ݣ�data_len���ݳ���
  ����ֵ��0--����ʧ�� 1--�����ɹ�
*/
uint8_t Parse_Refere_System_Data(uint8_t *get_data, uint16_t data_len)
{
	uint8_t a5_position[8];	 // 0xA5��λ��
	uint8_t a5_number = 0;	 // 0xA5�ĸ��������ݰ�������
	uint16_t data_length[8]; //ÿ��data���ݰ��ĳ���

	//Ѱ��֡ͷ
	Find_All_A5(get_data, data_len, a5_position, &a5_number);
	//�����������ݰ�
	for (uint8_t i = 0; i < a5_number; i++)
	{
		//����֡ͷ����
		if (Analysis_Frame_Header(&get_data[(a5_position[i])], &data_length[i], NULL) == 0)
		{
			// debug_log("Analysis No.%d frame header error.", i);
			// Console.print("head_done%f\r\n", i);
			continue;
		}

		//����CRC16У��
		if (Check_Package_Crc16(&get_data[(a5_position[i])], (data_length[i] + 9)) == 0)
		{
			// debug_log("CRC16 check No.%d fail.", i);
			// Console.print("CRC_done\r\n");

			continue;
		}

		//���������ݰ�
		if (Analysis_Data(&get_data[(a5_position[i])], data_length[i]) == 0)
		{
			// debug_log("Analysis No.%d data fail.", i);
			// Console.print("par\r\n");

			continue;
		}

		// Console.print("x%d len:%d p:%d id:%d\r\n", i, data_length[i], a5_position[i], Analysis_Cmd_Id(&get_data[(a5_position[i])]));
	}

	return 1;
}

/*
  ��������Find_All_A5
  ����  ���ҵ�����֡ͷ��ͷ
  ����  ��get_data��Ҫ������֡ͷ���ݣ�data_len���ݳ��ȣ�r_position 0xA5λ�����飬r_a5_length�������鳤�ȵĳ���
  ����ֵ����
*/
void Find_All_A5(uint8_t *get_data, uint16_t data_len, uint8_t *r_position, uint8_t *r_a5_length)
{
	*r_a5_length = 0;

	for (uint16_t i = 0; i < data_len; i++)
	{
		if (get_data[i] == 0xA5)
		{
			r_position[*r_a5_length] = i;

			(*r_a5_length)++;
		}
	}
}

/*
  ��������Analysis_Frame_Header
  ����  ������֡ͷ����
  ����  ��get_data��Ҫ���������ݰ���r_data_length��������֡�� data �ĳ��ȣ�r_seq����seqֵ
  ����ֵ��0--����ʧ�� 1--�����ɹ�
*/
uint8_t Analysis_Frame_Header(uint8_t *get_data, uint16_t *r_data_length, uint8_t *r_seq)
{
	JUDGE_ARRAY("head", get_data, 5);

	//�ж�֡ͷ��ͷ�Ƿ�Ϊ0xA5
	if (get_data[0] != 0xA5)
	{
		DEBUG_ERROR(501);
		return 0;
	}

	//֡ͷCRC8У��
	if (Verify_CRC8_Check_Sum(get_data, 5) == 0)
	{
		DEBUG_ERROR(502);
		return 0;
	}

	//����data�ĳ���
	if (r_data_length != NULL)
	{
		*r_data_length = get_data[1] | (get_data[2] << 8);
	}

	//����seq(�����)
	if (r_seq != NULL)
	{
		*r_seq = get_data[3];
	}

	return 1;
}

/*
  ��������Check_Package_Crc16
  ����  ��һ�����ݰ�����CRC16У��
  ����  ��get_data��Ҫ���������ݰ��� data_len������
  ����ֵ��0--����ʧ�� 1--�����ɹ�
*/
uint8_t Check_Package_Crc16(uint8_t *get_data, uint16_t data_len)
{
	return Verify_CRC16_Check_Sum(get_data, data_len);
}

/*
  ��������Analysis_Cmd_Id
  ����  ����������ID����
  ����  ��get_data��Ҫ���������ݰ�
  ����ֵ��cmd_id
*/
uint16_t Analysis_Cmd_Id(uint8_t *get_data)
{
	return get_data[5] + (get_data[6] << 8);
}

/*
  ��������Analysis_Data
  ����  ������data����
  ����  ��get_data��Ҫ���������ݰ�, data_len data�ĳ���
  ����ֵ��0--����ʧ�� 1--�����ɹ�
*/
uint8_t Analysis_Data(uint8_t *get_data, uint16_t data_len)
{
	uint16_t cmd_id = get_data[5] | (get_data[6] << 8);

	switch (cmd_id)
	{
	//ʵʱ������������
	case Power_Heat_Data:
	{
		return Analysis_Power_Heat_Data(&get_data[7], data_len);
		// break;
	}
	//ʵʱ�����Ϣ
	case Shoot_Data:
	{
		return Analysis_Shoot_Data(&get_data[7], data_len);
		// break;
	}
	//ʵʱ����������״̬
	case Game_Robot_Status:
	{
		return Analysis_Game_Robot_Status(&get_data[7], data_len);
		// break;
	}

	case BUFF_MUSK_CMD_ID:
	{
		memcpy(&judge_data.ext_buff_musk_t,&get_data[7], data_len);
	}
	case BULLET_REMAINING_CMD_ID:
	{
		// memcpy(&judge_data.ext_bullet_remaining_t.bullet_remaining_num,&get_data[7], sizeof(judge_data.ext_bullet_remaining_t));
		memcpy(&judge_data.ext_bullet_remaining_t.bullet_remaining_num, &get_data[7], data_len);
	}
	}

	return 0x0A;
}

//����ʵʱ������������
static uint8_t Analysis_Power_Heat_Data(uint8_t *data_package, uint16_t data_len)
{
	DATA_LEN_CHECK(data_len, 16, 503);

	judge_data.power_heat_data.chassis_power = Hex4_To_Float1(&data_package[4]);
	judge_data.power_heat_data.chassis_power_buffer = Uint8_t_Array_To_Uint16_t(&data_package[8]);
	judge_data.power_heat_data.shooter_id1_17mm_cooling_heat = Uint8_t_Array_To_Uint16_t(&data_package[10]);
	// debug_showdata2("chassis_power", judge_data.power_heat_data.chassis_power);
	// Console.print("chassis_power", judge_data.power_heat_data.shooter_id1_17mm_cooling_heat);
	return 1;
}

/**
 * @note	unfinished
 * @brief	����ʵʱ�����Ϣ
 * @return	���������1�ɹ�������0����ʧ�ܣ�����У�鲻ͨ����
 * */
static uint8_t Analysis_Shoot_Data(uint8_t *data_package, uint16_t data_len)
{
	DATA_LEN_CHECK(data_len, 7, 504);

	memcpy(&judge_data.shoot_data, data_package, 7);

	/**
	 * @note	unfinished
	 */
	//  Shooter_Friction_Speed_Limit();///<�����ӵ������ٶȺ����������ƣ�����������ϵͳ����
	debug_showdata2("bullet_speed", judge_data.shoot_data.bullet_speed);

	return 1;
}

/**
 * @brief		��������������״̬
 * @param[in]	���ݰ������׵�ַ����Ӧƫ����Ϊ0��
 * @param[in]	���ݳ���
 * @return		�ɹ�������1 &���ݳ��Ȳ���������0���Ҵ�ӡ��������505
 *  */
static uint8_t Analysis_Game_Robot_Status(uint8_t *data_package, uint16_t data_len)
{
	DATA_LEN_CHECK(data_len, 27, 505);

	judge_data.game_robot_status.robot_id = data_package[0];

	//������ 42mm ǹ��ÿ����ȴֵ
	judge_data.game_robot_status.shooter_id1_42mm_cooling_rate = Uint8_t_Array_To_Uint16_t(&data_package[18]);

	//������ 42mm ǹ����������
	judge_data.game_robot_status.shooter_id1_42mm_cooling_limit = Uint8_t_Array_To_Uint16_t(&data_package[20]);

	//������ 42mm ǹ�������ٶ� ��λ m/s
	judge_data.game_robot_status.shooter_id1_42mm_speed_limit = Uint8_t_Array_To_Uint16_t(&data_package[22]);
	/**
	 * @note	unfinished
	 */
	// Shooter_Friction_Speed_Base_Limit(judge_data.game_robot_status.shooter_id1_17mm_speed_limit); //���ٱ��ʱ������Ħ�����ٶ�

	//���̹�������
	judge_data.game_robot_status.chassis_power_limit = Uint8_t_Array_To_Uint16_t(&data_package[24]);

	judge_data.game_robot_status.mains_power_shooter_output = (data_package[26] & 0x04) >> 2;

	// //ģ�ⷢ������ϵ�
	// if(judge_data.game_robot_status.mains_power_shooter_output)
	// {
	// 	POWER1_CTRL_ON;
	// 	POWER2_CTRL_ON;
	// }
	// else
	// {
	// 	POWER1_CTRL_OFF;
	// 	POWER2_CTRL_OFF;
	// 	Fric_Reset(); //����Ħ����
	// }

	debug_showdata1("sout", judge_data.game_robot_status.mains_power_shooter_output);
	debug_showdata1("gggglllll", judge_data.game_robot_status.chassis_power_limit);

	return 1;
}

//�ж�42mm��������Ƿ�����
uint8_t Is_Id1_17mm_Excess_Heat(const Judge_data_t *judge_data)
{
	if (judge_data->power_heat_data.shooter_id1_42mm_cooling_heat == 65535 || Get_Module_Online_State(5) == 0)
	{
		return 0;
	}
	if (judge_data->power_heat_data.shooter_id1_42mm_cooling_heat <= (judge_data->power_heat_data.shooter_id1_42mm_cooling_heat + 12))
	{
		return 1;
	}
	return 0;
}
