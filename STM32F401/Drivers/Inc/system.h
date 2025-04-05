#ifndef INC_SYSTEM_H_
#define INC_SYSTEM_H_

#include "stm32f401xx.h"

// Define led
#define LED_PORT        GPIOB
#define LED_PIN         GPIO_PIN_NO_1

uint64_t system_get_ticks(void);
void system_init(void);

#endif /* INC_SYSTEM_H_ */