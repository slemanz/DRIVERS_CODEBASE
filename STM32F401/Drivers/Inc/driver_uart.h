#ifndef DRIVER_UART_H_
#define DRIVER_UART_H_

#include "stm32f401xx.h"

// Define the callback function type
typedef void (*uart_callback_t)(uint8_t data);

/*
 *  Bit position definions Uart
 */

#define UART_CR1_UE                 13
#define UART_CR1_M                  12
#define UART_CR1_PCE                10
#define UART_CR1_RXNEIE             5
#define UART_CR1_TE                 3
#define UART_CR1_RE                 2

#define UART_CR2_STOP               12

#define UART_SR_CTS                 9
#define UART_SR_TXE                 7
#define UART_SR_TC                  6
#define UART_SR_RXNE                5
#define UART_SR_ORE                 3




#define UART_CR1_UE_MASK             (1 << UART_CR1_UE)
#define UART_CR1_M_MASK              (1 << UART_CR1_M)
#define UART_CR1_PCE_MASK            (1 << UART_CR1_PCE)
#define UART_CR1_RXNEIE_MASK         (1 << UART_CR1_RXNEIE)
#define UART_CR1_TE_MASK             (1 << UART_CR1_TE)
#define UART_CR1_RE_MASK             (1 << UART_CR1_RE)

#define UART_CR2_STOP_MASK           (1 << UART_CR2_STOP)

#define UART_SR_CTS_MASK             (1 << UART_SR_CTS)
#define UART_SR_TXE_MASK             (1 << UART_SR_TXE)
#define UART_SR_TC_MASK              (1 << UART_SR_TC)
#define UART_SR_RXNE_MASK            (1 << UART_SR_RXNE)
#define UART_SR_ORE_MASK             (1 << UART_SR_ORE)

/*
 *  UART related status flags definitions
 */


#define UART_FLAG_CTS	                (UART_SR_TXE_CTS)
#define UART_FLAG_TXE	                (UART_SR_TXE_MASK)
#define UART_FLAG_TC                 	(UART_SR_TC_MASK)
#define UART_FLAG_RXNE              	(UART_SR_RXNE_MASK)
#define UART_FLAG_ORE                 	(UART_SR_ORE_MASK)


void uart2_init(void);
void uart2_init_pins(void);
void uart2_interrupt_enable(void);

int uart2_write(int ch);
int uart2_read(void);

void uart2_CallbackRegister(uart_callback_t callback);

#endif