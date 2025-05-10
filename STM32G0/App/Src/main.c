#include "stm32g0.h"
#include "config.h"
#include <stdio.h>


int main(void)
{
    config_drivers();

    GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_5, GPIO_PIN_SET);

    while(1)
    {
    }
}

