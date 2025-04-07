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
    UART2_PCLK_EN();

    // no flow control (default reset)
    uint32_t temp = ((1 << 3) | (1 << 2)); // tx and rx enable
    UART2->CR1 = temp; 
    UART2->BRR = compute_uart_div(16000000, 115200); // baurate

    UART2->CR1 |= (1 << 13);// enable uart periph
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

void UART_PeriClockControl(UART_RegDef_t *pUARTx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        if(pUARTx == UART1)
        {
            UART1_PCLK_EN();
        }else if(pUARTx == UART2)
        {
            UART2_PCLK_EN();
        }else if(pUARTx == UART6)
        {
            UART6_PCLK_EN();
        }
    }else
    {
        if(pUARTx == UART1)
        {
            UART1_PCLK_DI();
        }else if(pUARTx == UART2)
        {
            UART2_PCLK_DI();
        }else if(pUARTx == UART6)
        {
            UART6_PCLK_DI();
        }
    }
}


/*
 * Init and De-init
 */

void UART_Init(UART_Config_t *pUARTConfig)
{
    // temporary variable
	uint32_t tempreg = 0;

	UART_PeriClockControl(pUARTConfig->pUSARTx, ENABLE);

	// Enable USART Tx and Rx engines
	if(pUARTConfig->USART_Mode == UART_MODE_ONLY_RX)
	{
		tempreg |= UART_CR1_RE_MASK;
	}else if(pUARTConfig->USART_Mode == UART_MODE_ONLY_TX)
	{
		tempreg |= UART_CR1_TE_MASK;
	}else if(pUARTConfig->USART_Mode == UART_MODE_TXRX)
	{
		tempreg |= (UART_CR1_RE_MASK | UART_CR1_TE_MASK);
	}

	if(pUARTConfig->USART_ParityControl == UART_PARITY_EN_EVEN)
	{
		tempreg |= UART_CR1_PCE_MASK;
	}else if(pUARTConfig->USART_ParityControl == UART_PARITY_EN_ODD)
	{
		tempreg |= UART_CR1_PCE;
		tempreg |= UART_CR1_PS;
	}

	// program the CR1 register
	pUARTConfig->pUSARTx->CR1 = tempreg;



	// *** Configuration of CR2 **************************************
	tempreg = 0;

	tempreg |= (pUARTConfig->USART_NoOfStopBits << UART_CR2_STOP);

	// program the CR2 register
	pUARTConfig->pUSARTx->CR2 = tempreg;

    // Config Baudrate
    pUARTConfig->pUSARTx->BRR = compute_uart_div(clock_getValue(), pUARTConfig->USART_Baud);

    UART_PeripheralControl(pUARTConfig->pUSARTx, ENABLE);
}
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
void UART_PeripheralControl(UART_RegDef_t *pUARTx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        pUARTx->CR1 |=  UART_CR1_UE_MASK;
    }else
    {
        pUARTx->CR1 &= ~UART_CR1_UE_MASK;
    }
}

uint8_t UART_GetFlagStatus(UART_RegDef_t *pUARTx , uint8_t FlagName);
void UART_ClearFlag(UART_RegDef_t *pUARTx, uint16_t StatusFlagName);