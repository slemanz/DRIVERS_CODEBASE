/*
 * ==========================================================================================
 *      File:   driver_gpio.c
 *      Author: William Sleman
 *
 *      Description:
 *      This source file contains the implementation of functions for configuring and 
 *      managing General Purpose Input/Output (GPIO) pins on the microcontroller. It 
 *      includes functionalities for enabling or disabling the GPIO peripheral clock, 
 *      initializing GPIO pins with specific modes, speeds, pull-up/pull-down settings, 
 *      output types, and alternate functions. Additionally, it provides functions to 
 *      read from input pins, write to output pins, and toggle output pin states.
 *
 *      Note:
 *      For details on the structures and function prototypes, see the header file: 
 *      driver_gpio.h.
 * ==========================================================================================
 */
#include "driver_gpio.h"


/*
 ********************************************************************************************
 * @fn              GPIO_PeriClockControl
 *
 * @brief           - Enables or disables the peripheral clock for the specified GPIO port.
 *
 * @param[in]       - pGPIOx: Pointer to the GPIO peripheral to control.
 * @param[in]       - EnorDi: Specify whether to enable or disable the clock (ENABLE/DISABLE).
 *
 * @return          - none
 *
 * @Note            - none
 ********************************************************************************************
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if		(pGPIOx == GPIOA) GPIOA_PCLK_EN();
		else if	(pGPIOx == GPIOB) GPIOB_PCLK_EN();
		else if	(pGPIOx == GPIOC) GPIOC_PCLK_EN();
	}else
	{
		if		(pGPIOx == GPIOA) GPIOA_PCLK_DI();
		else if	(pGPIOx == GPIOB) GPIOB_PCLK_DI();
		else if	(pGPIOx == GPIOC) GPIOC_PCLK_DI();
	}
}


/*
 ********************************************************************************************
 * @fn              - GPIO_Init
 *
 * @brief           - Initializes the specified GPIO pin based on the provided configuration.
 *
 * @param[in]       - pGPIOHandle: Pointer to the GPIO handle structure containing the pin configuration.
 *
 * @return          - none
 *
 * @Note            - This function enables the clock for the GPIO port and configures the mode,
 *                    speed, pull-up/pull-down settings, output type, and alternate function
 *                    settings for the specified GPIO pin.
 ********************************************************************************************
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
    GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	uint32_t temp = 0;

	// 1. configure the mode of the gpio pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
		pGPIOHandle->pGPIOx->MODER |= temp; // setting
	}else
	{

	}

	temp = 0;

	// 2. configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;


	// 3. configure the pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;


	// 4. configure the output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;


	// 5. configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint32_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0x0F << (4*temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2));
	}
}


/*
 ********************************************************************************************
 * @fn              - GPIO_DeInit
 *
 * @brief           - Deinitializes the specified GPIO port by resetting its registers.
 *
 * @param[in]       - pGPIOx: Pointer to the GPIO peripheral to deinitialize.
 *
 * @return          - none
 *
 * @Note            - none
 ********************************************************************************************
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
}


/*
 ********************************************************************************************
 * @fn              - GPIO_ReadFromInputPin
 *
 * @brief           - Reads the value of a specified GPIO input pin.
 *
 * @param[in]       - pGPIOx: Pointer to the GPIO peripheral to read from.
 * @param[in]       - PinNumber: The pin number of the GPIO input to read.
 *
 * @return          - uint8_t: The value read from the specified pin (0 or 1).
 *
 * @Note            - none
 ********************************************************************************************
 */
uint8_t  GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x01);

	return value;
}


/*
 ********************************************************************************************
 * @fn              - GPIO_WriteToOutputPin
 *
 * @brief           - Writes a value to a specified GPIO output pin.
 *
 * @param[in]       - pGPIOx: Pointer to the GPIO peripheral to write to.
 * @param[in]       - PinNumber: The pin number of the GPIO output to write.
 * @param[in]       - Value: The value to write to the specified pin (GPIO_PIN_SET or GPIO_PIN_RESET).
 *
 * @return          - none
 *
 * @Note            - none
 ********************************************************************************************
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}


/*
 ********************************************************************************************
 * @fn              - GPIO_ToggleOutputPin
 *
 * @brief           - Toggles the state of a specified GPIO output pin.
 *
 * @param[in]       - pGPIOx: Pointer to the GPIO peripheral to toggle.
 * @param[in]       - PinNumber: The pin number of the GPIO output to toggle.
 *
 * @return          - none
 *
 * @Note            - none
 ********************************************************************************************
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}