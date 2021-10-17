#include "console_print.h"

/**
 * @brief ����ֱ��ʹ�� Console.print() ��������ͨ�� printf ����
 */
const Console_t Console = {
    .print = __printf,
    .log = __printf_log,
    .error = __printf_error,
    .warning = __printf_warning
};
