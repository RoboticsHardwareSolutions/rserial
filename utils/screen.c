#include "screen.h"
#include <stdio.h>
#include <stdarg.h>

void clean_screen(void)
{
    printf("\033[2J");
    printf("\033[H");
}

void start_screen(void)
{
    puts("\033[?25l");
    printf("\033[H");
}

void println(char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    printf("\033[0K\n");
}

void end_screen(void)
{
    puts("\033[0J");
    puts("\033[?25h");
}
