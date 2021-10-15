#ifndef __CONSOLE_PRINT_H_
#define __CONSOLE_PRINT_H_

#include "usart3.h"

/**
 * @brief 串口打印函数指针结构体
 */
typedef struct
{
    void (*log)(const char *, ...);
    void (*warning)(const char *, ...);
    void (*error)(const char *, ...);

} Console_t;

extern const Console_t Console;

#endif //__CONSOLE_PRINT_H_
