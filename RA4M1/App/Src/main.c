// Author: William Sleman @ 2025
#include <stdint.h>

#define GPIO_P0_BASEADDR        0x40040000U
#define MMIO32(addr) (*(volatile uint32_t *)(addr))

// Led   -> P111_GPT3_A
const uint8_t leds_pin[] = {0, 6, 16, 24};

void delay_cycles(uint32_t cycles)
{
    while (cycles-- > 0) {
        __asm("NOP"); // No operation for delay
    }
}

int main(void)
 {
    MMIO32(GPIO_P0_BASEADDR) |= (1 << 11); // set as output
    MMIO32(GPIO_P0_BASEADDR) |= (1 << 27); // toggle

    while (1)
    {   
        //MMIO32(GPIO_P0_BASEADDR) ^= (1 << 27); // toggle
        delay_cycles(5000000);
    }
}