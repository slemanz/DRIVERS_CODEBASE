#ifndef DRIVER_SYSTICK_H_
#define DRIVER_SYSTICK_H_

#include "stm32f401xx.h"

#define TICK_HZ                 1000U
#define SYSTICK_TIM_CLK			HSI_CLOCK

void systick_init(uint32_t tick_hz);

#endif