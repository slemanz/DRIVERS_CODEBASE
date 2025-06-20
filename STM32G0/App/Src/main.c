#include "stm32g0.h"
#include "config.h"
#include <stdio.h>


int main(void)
{
    config_drivers();
    GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_5, GPIO_PIN_SET);

    uint64_t start_time = ticks_get();

    while(1)
    {
        //blinky
        if((ticks_get() - start_time) >= 500)
        {
            GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
            start_time = ticks_get();
        }
    }
}

