#include "config.h"
#include "common-defines.h" 
#include "comms/serial.h"

#include "driver_gpio.h"
#include "driver_systick.h"



int main(void)
{
    config_drivers();
    config_comms();
    uint8_t data_send[] = "\nINIT\n\r";
    serial_send(data_send, 7);

    uint64_t start_time = systick_get();
    uint64_t start_time2 = systick_get();


    GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_2, GPIO_PIN_RESET);
    GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_5, GPIO_PIN_SET);

    while (1)
    {
        if((systick_get() - start_time) >= 1000)
        {
            GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
            GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_2);
            start_time = systick_get();
        }

        if((systick_get() - start_time2) >= 10000)
        {
            serial_send_byte('a');
            start_time2 = systick_get();
        }
    }
}
