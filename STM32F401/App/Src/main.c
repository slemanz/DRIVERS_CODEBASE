#include "stm32f401xx.h"
#include <stdio.h>
#include "common-defines.h" 
#include "system.h"

void handle_received_data(uint8_t data);

float cb_square(int num)
{
    return num*num;
}

float cb_cube(int num)
{
    return num*num*num;
}

typedef float (*op_t)(int);

float calc(int num, op_t op)
{
    return op(num);
}

uint8_t data_rcv = 0;

int main(void)
{
    system_init();
    printf("\n");
    printf("Init..\n");

    printf("2^2 = %.1f\n\r", calc(2, cb_square));
    printf("2^3 = %.1f\n\r", calc(2, cb_cube));

    uart2_CallbackRegister(handle_received_data);
    uart2_interrupt_enable();
    interrupt_config(IRQ_NO_UART2, ENABLE);


    while (1)
    {
        if(data_rcv != 0)
        {
            uart2_write((int)data_rcv);
            data_rcv = 0;
        }
    }
}

void handle_received_data(uint8_t data)
{
    // Process received data here
    data_rcv = data;
}