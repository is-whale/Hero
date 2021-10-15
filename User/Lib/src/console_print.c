#include "console_print.h"

void _log(const char *,...);
void _warning(const char *,...);
void _error(const char *,...);

const Console_t Console = {
    .log = _log,
    .error = _error,
    .warning = _warning
};

void _log(const char *fmt,...)
{
    __printf(">>LOG>>  ");
    __printf(fmt);
}

void _warning(const char *fmt,...)
{
    __printf(">>WARNING>>  ");
    __printf(fmt);
}
void _error(const char *fmt,...)
{
    __printf(">>ERROR>>  ");
    __printf(fmt);
}