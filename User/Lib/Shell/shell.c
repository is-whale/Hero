#include "shell.h"
#include "shellio.h"
#include "user_commands.h"

union cmd_id
{
    struct
    { // ����ŷ�Ϊ�����������
        unsigned int num : 8;        ///< �������
        unsigned int sum : 8;        ///< �����ַ����ܺ�
        unsigned int len : 8;        ///< �����ַ��ĳ��ȣ�8 bit ��������Ȳ��ܳ���31���ַ�
        unsigned int first_char : 8; ///< �����ַ��ĵ�һ���ַ�
    } part;

    unsigned int id; ///< �ɴ˺ϲ�Ϊ 32 λ��������
};

Shell_command_t shell_cmd_root;


/**
  * @author   Bashpow
  * @brief    ����������¼
  * @author   Bashpow
  * @param    root     : ������������ͷ�ڵ�
  * @param    newcmd   : ��������ƿ�
  * @return   �ɹ����� 0
*/
static int Shell_Insert_Cmd(Shell_command_t * root , Shell_command_t * newcmd)
{
	Shell_command_t *node = root ;

	while( node->next != NULL )
	{
		if(newcmd->id == (node->next)->id)
			return -1;

		else if (newcmd->id < (node->next)->id)
			break;

		node = node->next;
	}

	newcmd->next = node->next;
	node->next = newcmd;

	root->id ++;
	return 0;
}

/**
  * @author   Bashpow
  * @brief    ע��һ������źͶ�Ӧ������� 
  * @note     ǰ׺Ϊ '_' ��ʾ������ֱ�ӵ��ô˺�����ͨ������ڵĺ궨��������
  * @param    root     : ��������ͷ
  * @param    new_cmd   : ������ƿ��Ӧ��ָ��
  * @param    cmd_name : ������
  * @param    cmd_func : ��������Ӧ��ִ�к���
  * @return   ��
*/
void _Shell_Register(Shell_command_t * root, Shell_command_t * new_cmd, char * cmd_name, cmd_callback_func cmd_func)
{
	union cmd_id new_cmd_id;
	unsigned int tmp_sum = 0;
	unsigned int tmp_len = 0;

	for(; cmd_name[tmp_len] != '\0'; tmp_len++)
	{
		tmp_sum += cmd_name[tmp_len];
	}

	new_cmd_id.part.num = root->id + 1;
	new_cmd_id.part.sum = tmp_sum;
	new_cmd_id.part.len = tmp_len;
	new_cmd_id.part.first_char = cmd_name[0];

	new_cmd->id = new_cmd_id.id;
	new_cmd->name = cmd_name;
	new_cmd->func = cmd_func;
	new_cmd->next = NULL;

	Shell_Insert_Cmd(root, new_cmd); //��������������
}

/**
  * @author   Bashpow
  * @brief    ��������
  * @param    arg  : �������ڴ�
  * @return   ��
*/
void Shell_Command_Parse(char * arg)
{
	Shell_command_t *node = &shell_cmd_root ;
	union cmd_id node_id;

	while( node->next != NULL )
	{
		node = node->next;
		node_id.id = node->id;

		if(STR_COMPARE(node->name, arg, node_id.part.len))
		{
			if ((arg[node_id.part.len] == '\0') || (arg[node_id.part.len] == ' '))
			{
				shell_print("rov> %s\r\n", arg);
				node->func(arg); //ִ�лص�����
				shell_print("\r\n");
				return;
			}
		}
	}

}

/**
  * @author   Bashpow
  * @brief    eg.�� "abc bc cdef xyz" ���Ϊ char*argv[] = {"abc","bc","cdef","xyz"};
  * @note     ���й����ַ������������ݽ����޸�
  * @param    str    : �����ַ���������������������ָ��
  * @param    argv   : ����ת���󻺴��ַ
  * @param    maxread: ����ȡ����һ��Ϊargv�ĸ�����
  * @return   ���ն�ȡ�����������
*/
int Shell_Split_String(char * str ,char ** argv ,int maxread)
{
	int argc = 0;

	for (; ' ' == *str; ++str)
		; //�����ո�

	for (; *str && argc < maxread; ++argc, ++argv) //�ַ���Ϊ ��\0' ��ʱ��
	{

		for (*argv = str; ' ' != *str && *str; ++str)
			; //��¼���������Ȼ�������ǿ��ַ�

		for (; ' ' == *str; *str++ = '\0')
			; //ÿ���������ַ����������������ո�
	}

	return argc;
}


/**
  * @author   Bashpow
  * @brief    ��ʾ����ע���˵�����
  * @param    arg  : �������ڴ�
  * @return   ��
*/
#ifndef VERSION
	#define VERSION "-"
#endif
static void Infantry_Version(char * arg)
{
	shell_print("\r\n");
	shell_print("    __  ____    __       _____                          __  __              \r\n");
	shell_print("   / / / / /   / /      / ___/__  ______  ___  _____   / / / /__  _________ \r\n");
	shell_print("  / /_/ / /   / /       \\__ \\/ / / / __ \\/ _ \\/ ___/  / /_/ / _ \\/ ___/ __ \r\n");
	shell_print(" / __  / /___/ /___    ___/ / /_/ / /_/ /  __/ /     / __  /  __/ /  / /_/ /\r\n");
	shell_print("/_/ /_/_____/_____/   /____/\\__,_/ .___/\\___/_/     /_/ /_/\\___/_/   \\____/ \r\n");
	shell_print("                                /_/                                         \r\n");
	shell_print("\r\n");
	shell_print("\r\n");
	shell_print("version:\t%s\r\n", VERSION);
}


/**
  * @author   Bashpow
  * @brief    ��ʾ����ע���˵�����
  * @param    arg  : �������ڴ�
  * @return   ��
*/
static void Shell_List_Cmd(char * arg)
{
	Shell_command_t *node = &shell_cmd_root ;
	shell_print("command----------\r\n");
	while( node->next != NULL )
	{
		node = node->next;
		shell_print("id:%d, name:%s\r\n", node->id, node->name);
	}
	shell_print("-----------------\r\n");
}

/**
  * @author   Bashpow
  * @brief    shell ��ʼ��,ע�Ἰ�����������ע���û���������鲻��ʼ����
  * @return   ��
*/
void Shell_Init(void)
{
	{
		shell_cmd_root.id = 0;
		shell_cmd_root.func = NULL;
		shell_cmd_root.next = NULL;
	}
	
	/* ע��һЩ�������� */
	Shell_Register_Command("version",       Infantry_Version);
	Shell_Register_Command("command-list" , Shell_List_Cmd);
	Shell_Register_Command("?" ,            Shell_List_Cmd);

	/* ע���û����� */
	User_Commands_Init();
}
