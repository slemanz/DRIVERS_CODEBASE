// Author: William Sleman @ 2025
#include <stdint.h>

#define PMISC_PWPR 		0x40040D03U

#define P100PFS 		0x40040840U
#define P111PFS 		(P100PFS + 0x2CU)

#define GPIO_P1_BASEADDR        0x40040000U
#define MMIO32(addr) (*(volatile uint32_t *)(addr))
#define MMIO8(addr)  (*(volatile uint8_t *)(addr))

// Led   -> P111_GPT3_A
const uint8_t leds_pin[] = {0, 6, 16, 24};

void delay_cycles(uint32_t cycles)
{
    while (cycles-- > 0)
    {
        __asm("NOP"); // No operation for delay
    }
}

int main(void)
{
    MMIO8(PMISC_PWPR) &= ~(1 << 7);
    MMIO8(PMISC_PWPR) |=  (1 << 6);

    MMIO32(P111PFS) |= (1 << 2); // output
    MMIO32(P111PFS) |= (1 << 0); // output
    //MMIO32(GPIO_P1_BASEADDR) |= (1 << 11); // set as output
    //MMIO32(GPIO_P1_BASEADDR) |= (1 << 27); // toggle

    MMIO32(PMISC_PWPR) &= ~(1 << 6);
    MMIO32(PMISC_PWPR) |=  (1 << 7);

    while (1)
    {   
        //MMIO32(GPIO_P1_BASEADDR) ^= (1 << 27); // toggle
        delay_cycles(5000000);
    }
}