#include "stm32g0.h"


int main(void)
{
    RCC->IOPENR |= (1 << 0);

    GPIOA->MODER &= ~(1 << 11);
    GPIOA->MODER |=  (1 << 10);

    GPIOA->ODR |= (1 << 5);

    while(1)
    {
        //blinky
        GPIOA->ODR ^= (1 << 5);
        for(uint32_t i = 0; i < 1000000; i++);
    }
}

