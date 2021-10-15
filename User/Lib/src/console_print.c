#include "console_print.h"

void _log(const char *,...);
void _warning(const char *,...);
void _error(const char *,...);

const Console_t Console = {
    .log = __printf,
    .error = _error,
    .warning = _warning
};

/**
 * @brief   ������־��ӡ
 * @note    ֱ�ӵ��ô��� 3 ��װ�õ� __printf ����
 * @param   fmt 
 * @param   ... 
 */
void _log(const char *fmt,...)
{
    __printf(">>LOG>>  ");
}

/**
 * @brief   ���� warning ��ӡ
 * @note    ֱ�ӵ��ô��� 3 ��װ�õ� __printf ����
 * @param   fmt 
 * @param   ... 
 */
void _warning(const char *fmt,...)
{
    __printf(">>WARNING>>  ");
    __printf(fmt);
}

/**
 * @brief   ���� error ��ӡ
 * @note    ֱ�ӵ��ô��� 3 ��װ�õ� __printf ����
 * @param   fmt 
 * @param   ... 
 */
void _error(const char *fmt,...)
{
    __printf(">>ERROR>>  ");
    __printf(fmt);
}
