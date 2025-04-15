#include "config.h"
#include "common-defines.h" 
#include <stdio.h>
#include "comms/serial.h"

#include "driver_gpio.h"

uint64_t ticks = 0;

uint64_t ticks_get()
{
    return ticks;
}


int main(void)
{
    config_drivers();
    config_comms();
    printf("\n\rInit\n\r");

    uint64_t start_time = ticks_get();
    uint64_t start_time2 = ticks_get();


    GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_2, GPIO_PIN_RESET);
    GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_5, GPIO_PIN_SET);

    while (1)
    {
        if((ticks_get() - start_time) >= 500)
        {
            GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
            GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_2);
            start_time = ticks_get();
        }

        if((ticks_get() - start_time2) >= 1000)
        {
            serial_send_byte('a');
            start_time2 = ticks_get();
        }
    }
}

void SysTick_Handler(void)
{
    ticks++;
}