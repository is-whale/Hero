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
 * @brief   串口日志打印
 * @note    直接调用串口 3 封装好的 __printf 函数
 * @param   fmt 
 * @param   ... 
 */
void _log(const char *fmt,...)
{
    __printf(">>LOG>>  ");
}

/**
 * @brief   串口 warning 打印
 * @note    直接调用串口 3 封装好的 __printf 函数
 * @param   fmt 
 * @param   ... 
 */
void _warning(const char *fmt,...)
{
    __printf(">>WARNING>>  ");
    __printf(fmt);
}

/**
 * @brief   串口 error 打印
 * @note    直接调用串口 3 封装好的 __printf 函数
 * @param   fmt 
 * @param   ... 
 */
void _error(const char *fmt,...)
{
    __printf(">>ERROR>>  ");
    __printf(fmt);
}
