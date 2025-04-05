#include "system.h"
#include <stdio.h>

void system_init(void)
{
    // Set GPIOA pin 3 as output
}

// printf retarget
extern int __io_putchar(int ch)
{
    return uart2_write(ch);
}

extern int __io_getchar(void)
{
    return uart2_read();
}
