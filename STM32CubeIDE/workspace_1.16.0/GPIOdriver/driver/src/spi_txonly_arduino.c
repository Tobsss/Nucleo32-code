/*
 * spi_tx_testing.c
 *
 *  Created on: Nov 26, 2024
 *      Author: User
 */

#include "stm32l476xx.h"
#include <string.h>
#include "stm32l476xx_spi_driver.h"
#include "stm32l476xx_gpio_driver.h"

void delay(void)
{
	for (uint32_t i = 0; i<25000 ; i ++);
}

/*
 * PB14-->SPI2_MISO
 * PB15-->SPI2_MOSI
 * PB13-->SPI2_SCLK
 * PB12-->SPI2_NSS
 * ALT function mode : 5
 */

void SPI_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
}

void SPI2_Inits(void){
	SPI_Handle_t SPI2handle;
	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32; //generates sclk of 8MHZ
	SPI2handle.SPI_Config.SPI_DFF = SPI_DFF_16BITS;
	SPI2handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPI_Config.SPI_SSM = SPI_SSM_DI; //software slave management enabled for NSS pin

	SPI_Init(&SPI2handle);

}
void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn;
	GPIOBtn.pGPIOx = GPIOC;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Init(&GPIOBtn);

}
int main(void)
{

	(RCC->APB1ENR1 |= (1 << 14));



	void GPIO_ButtonInit(void);

	//This function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI_GPIOInits();

	GPIOB->AFRH |= ((1<<30)|(1<<28)|(1<<22)|(1<<20)|(1<<18)|(1<<16)|(1<<26)|(1<<24));
	//This function is used to initialize the SPI2 peripherals parameters

	SPI2_Inits();
	char user_data[] = "Hello";
	SPI_SSOEConfig(SPI2, ENABLE);
	while (1)
	{
		//wait till button is pressed
		while (!GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13));

		//delay();

		//enable the SPI2 peripheral
		//SPI_SSIConfig(SPI2, ENABLE);
		SPI_PeripheralControl(SPI2, ENABLE);
		//uint8_t dataLen = strlen(user_data);
		//SPI_SendData(SPI2, &dataLen,1);
		//to send data
		//SPI2-> DR = 'A';
		//while(!(SPI2->SR) & (1<<1));
		SPI_SendData(SPI2, (uint16_t*)user_data, strlen(user_data));

		while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));

	    SPI_PeripheralControl(SPI2, DISABLE);
	}

	return 0;
}
