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
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(&SPIPins);

	//NSS
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	//GPIO_Init(&SPIPins);
}

void SPI2_Inits(void){
	SPI_Handle_t SPI2handle;
	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; //generates sclk of 8MHZ
	SPI2handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPI_Config.SPI_CPOL = SPI_CPOL_HIGH;
	SPI2handle.SPI_Config.SPI_CPHA = SPI_CPHA_HIGH;
	SPI2handle.SPI_Config.SPI_SSM = SPI_SSM_EN; //software slave management enabled for NSS pin

	SPI_Init(&SPI2handle);

}

int main(void)
{
	//GPIOB->AFRH |= ((1<<28)|(1<<30)|(1<<22)|(1<<20));
	(RCC->APB1ENR1 |= (1 << 14));
	char user_data[] = "Hello World";
	//This function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI_GPIOInits();
	GPIOB->AFRH |= ((1<<28)|(1<<30)|(1<<22)|(1<<20));
	//This function is used to initialize the SPI2 peripherals parameters

	SPI2_Inits();

	//enable the SPI2 peripheral
	SPI_SSIConfig(SPI2, ENABLE);
	SPI_PeripheralControl(SPI2, ENABLE);

	//to send data
	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));
	while(1);

	return 0;
}
