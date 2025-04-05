#include "config.h"
#include "stm32f401xx.h"

static void config_gpio(void)
{
    GPIO_Handle_t GpioLed;
	GpioLed.pGPIOx = GPIOB;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_Init(&GpioLed);
    GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_2, GPIO_PIN_SET);
}


void config_drivers(void)
{
    while(clock_init());

    config_gpio();
    uart2_init();
}