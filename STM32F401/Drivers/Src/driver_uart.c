#include "driver_uart.h"

// Global variable to store the callback
static uart_callback_t uart2_rx_callback = NULL;

static uint16_t compute_uart_div(uint32_t PeriphClk, uint32_t BaudRate);

static uint16_t compute_uart_div(uint32_t PeriphClk, uint32_t BaudRate)
{
    return ((PeriphClk + (BaudRate/2U))/BaudRate);
}


void uart2_init(void)
{
    uart2_init_pins();
    UART2_PCLK_EN();

    // no flow control (default reset)
    uint32_t temp = ((1 << 3) | (1 << 2)); // tx and rx enable
    UART2->CR1 = temp; 
    UART2->BRR = compute_uart_div(16000000, 115200); // baurate

    UART2->CR1 |= (1 << 13);// enable uart periph
}


void uart2_init_pins(void)
{
    /*
    GPIO_Handle_t UartPin;

    UartPin.pGPIOx = GPIOA;
    UartPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    UartPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
    UartPin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    UartPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    UartPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    UartPin.GPIO_PinConfig.GPIO_PinAltFunMode = PA2_ALTFN_UART2_TX;

    GPIO_Init(&UartPin);

    UartPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
    UartPin.GPIO_PinConfig.GPIO_PinAltFunMode = PA3_ALTFN_UART2_RX;
    GPIO_Init(&UartPin);
    */
}

void uart2_interrupt_enable(void)
{
    UART2->CR1 &= ~(1 << 0);
    UART2->CR1 |=  (1 << 5);
    UART2->CR1 |=  (1 << 0);
}


int uart2_write(int ch)
{
	while(!(UART2->SR & UART_FLAG_TXE));
    UART2->DR = ch;

    return ch;
}

int uart2_read(void)
{
    return UART2->DR;
}

void uart2_CallbackRegister(uart_callback_t callback)
{
    uart2_rx_callback = callback;
}

void USART2_IRQHandler(void)
{
    const bool overrun_occurred = (UART2->SR & (1 << 3));
	const bool received_data = (UART2->SR & (1 << 5));

	if(received_data || overrun_occurred)
	{
        uint8_t received_data = uart2_read();
        // Call registered callback if exists
        if(uart2_rx_callback != NULL)
        {
            uart2_rx_callback(received_data);
        }
	}
}

// NEW API

void UART_PeriClockControl(UART_RegDef_t *pUSARTx, uint8_t EnorDi)
{

}


/*
 * Init and De-init
 */
void UART_Init(UART_Config_t *pUARTConfig);
void UART_DeInit(UART_RegDef_t *pUSARTx);


/*
 * Data Send and Receive
 */
void UART_write_byte(UART_RegDef_t *pUARTx, uint8_t data);
uint8_t UART_read_byte(UART_RegDef_t *pUARTx);

void UART_Write(UART_RegDef_t *pUARTx, uint8_t *pTxBuffer, uint32_t Len);
void UART_Read(UART_RegDef_t *pUARTx, uint8_t *pRxBuffer, uint32_t Len);


/*
 * IRQ Configuration and ISR handling
 */

void UART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void UART_CallbackRegister(UART_RegDef_t *pUARTx, uart_callback_t rx_callback, uart_callback_t tx_callback);
void uart2_CallbackRegister(uart_callback_t callback);


/*
 * Other Peripheral Control APIs
 */
void UART_PeripheralControl(UART_RegDef_t *pUARTx, uint8_t EnorDi);
uint8_t UART_GetFlagStatus(UART_RegDef_t *pUARTx , uint8_t FlagName);
void UART_ClearFlag(UART_RegDef_t *pUARTx, uint16_t StatusFlagName);
void UART_SetBaudRate(UART_RegDef_t *pUARTx, uint32_t BaudRate);