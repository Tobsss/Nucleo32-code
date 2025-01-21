/*
 * stm32l476xx_gpio_driver.c
 *
 *  Created on: Oct 25, 2024
 *      Author: User
 */


#include "stm32l476xx_gpio_driver.h"

/**************************************************
 * @fn				- GPIO_PeriClockControl
 *
 * @brief			- This function enables or disables peripheral clock for given GPIO port
 *
 * @param[in]		- base address of the gpio peripheral
 * @param[in]		- ENABLE or DISABLE macros
 * @param[in]
 *
 * @return			- none
 *
 * @Note			- none
 *
 *
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK;
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK;
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK;
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK;
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK;
		}else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK;
		}else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK;
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK;
		}else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK;
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
				{
					GPIOA_PCLK_DI;
				}else if (pGPIOx == GPIOB)
				{
					GPIOB_PCLK_DI;
				}else if (pGPIOx == GPIOC)
				{
					GPIOC_PCLK_DI;
				}else if (pGPIOx == GPIOD)
				{
					GPIOD_PCLK_DI;
				}else if (pGPIOx == GPIOE)
				{
					GPIOE_PCLK_DI;
				}else if (pGPIOx == GPIOF)
				{
					GPIOF_PCLK_DI;
				}else if (pGPIOx == GPIOG)
				{
					GPIOG_PCLK_DI;
				}else if (pGPIOx == GPIOH)
				{
					GPIOH_PCLK_DI;
				}else if (pGPIOx == GPIOI)
				{
					GPIOI_PCLK_DI;
				}
	}
}

/**************************************************
 * @fn				- GPIO_Init
 *
 * @brief			- This function initializes the GPIO port
 *
 * @param[in]		- GPIO Handle struct
 * @param[in]		-
 * @param[in]
 *
 * @return			- none
 *o
 * @Note			- none
 *
 *l
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp=0x0; //temp. register

	//enable the peripheral clock

	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);
	//1 . configure the mode of gpio pin
	pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber))); //clear
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//the non interrupt mode
		temp = ((pGPIOHandle->GPIO_PinConfig.GPIO_PinMode) << (2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));

		pGPIOHandle->pGPIOx->MODER |= temp; //setting

	}else
	{
		//code this part later . (interrupt mode)
	}

	temp = 0;

	//4 . configure the optype
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clear
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;
	//2. configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clear
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;
	// configure bssr
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_Set << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->BSRR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clear
	pGPIOHandle->pGPIOx->BSRR |= temp;

	temp = 0;

	//3 . configure the pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clear
	pGPIOHandle->pGPIOx->PUPDR |= temp;



	//temp = 0;

	//5 . configure the alt functionality
/*	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		//CONFIGURE THE ALT FUNCTION REGISTERS.
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0XF << (4 * temp2)); //clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}*/


}

/**************************************************
 * @fn				- GPIO_PeriClockControl
 *
 * @brief			- This function enables or disables peripheral clock for given GPIO port
 *
 * @param[in]		- base address of the gpio peripheral
 * @param[in]		- ENABLE or DISABLE macros
 * @param[in]
 *
 * @return			- none
 *
 * @Note			- none
 *
 *
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
		if(pGPIOx == GPIOA)
		{
			GPIOA_REG_RESET();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_REG_RESET();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_REG_RESET();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_REG_RESET();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_REG_RESET();
		}else if (pGPIOx == GPIOF)
		{
			GPIOF_REG_RESET();
		}else if (pGPIOx == GPIOG)
		{
			GPIOG_REG_RESET();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_REG_RESET();
		}else if (pGPIOx == GPIOI)
		{
			GPIOI_REG_RESET();
		}
}

/*
 * Data read and write
 */
/**************************************************
 * @fn				- GPIO_ReadFromInputPin
 *
 * @brief			- This function enables input is read from pin
 *
 * @param[in]		- base address of the gpio peripheral
 * @param[in]		- Pin Number
 * @param[in]
 *
 * @return			- 0 or 1
 *
 * @Note			- none
 *
 *
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

/**************************************************
 * @fn				- GPIO_ReadFromInputPort
 *
 * @brief			- This function enables input is read from entire port
 *
 * @param[in]		- base address of the gpio peripheral
 * @param[in]		-
 * @param[in]
 *
 * @return			- 0 or 1
 *
 * @Note			- none
 *
 *
 */

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}

/**************************************************
 * @fn				- GPIO_WriteToOutputPin
 *
 * @brief			- This function enables output to be read from a pin
 *
 * @param[in]		- base address of the gpio peripheral
 * @param[in]		- Pin Number
 * @param[in]		- Value to be read
 *
 * @return			- none
 *
 * @Note			- none
 *
 *
 */

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,uint8_t value )
{
	if(value == GPIO_PIN_SET){
		//WRITE 1 to the output data register at the bit field corresponding to the pin
		pGPIOx->ODR |= (1 << PinNumber);
	}else
	{
		//write 0
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/**************************************************
 * @fn				- GPIO_WriteToOutputPort
 *
 * @brief			- This function enables output to be read from a port
 *
 * @param[in]		- base address of the gpio peripheral
 * @param[in]		- Pin Number
 * @param[in]		- Value to be read
 *
 * @return			- 0 or 1
 *
 * @Note			- none
 *
 *
 */

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	//pGPIOx->MODER |=(1 << (2 * PinNumber));
	pGPIOx->ODR |= (1 << PinNumber);
	delay(300);
	pGPIOx->ODR &= ~(1<<PinNumber);
	delay(1000);
}

/*
 * IRQ Configuration and ISR handling
 */

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi)
{

}
void GPIO_IRQHandling(uint8_t PinNumber)
{

}
