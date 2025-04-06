#include "config.h"
#include "stm32f401xx.h"

static const GPIO_Config_t gpioConfigs[] = {
    {GPIOB, GPIO_PIN_NO_2, GPIO_MODE_OUT, GPIO_SPEED_LOW, GPIO_OP_TYPE_PP, GPIO_NO_PUPD},
    {GPIOA, GPIO_PIN_NO_5, GPIO_MODE_OUT, GPIO_SPEED_LOW, GPIO_OP_TYPE_PP, GPIO_NO_PUPD}
    // Add more configurations as needed
};

static void config_gpio(void)
{
    for (uint32_t i = 0; i < sizeof(gpioConfigs) / sizeof(gpioConfigs[0]); i++)
    {
        GPIO_Init((GPIO_Config_t *)&gpioConfigs[i]);
    }
}


void config_drivers(void)
{
    while(clock_init());

    config_gpio();
    systick_init(1000, clock_getValue());
    //uart2_init();
}