/*
 * 001Ledtoggle.c
 *
 *  Created on: Oct 31, 2024
 *      Author: User
 */

#include "stm32l476xx.h"
#include "stm32l476xx_gpio_driver.h"



int main(void)
{

	GPIO_Handle_t GpioLed;
	GpioLed.pGPIOx = GPIOC;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	 uint32_t a = GpioLed.GPIO_PinConfig.GPIO_PinMode;
	 uint32_t b = GpioLed.GPIO_PinConfig.GPIO_PinNumber;


	GPIO_PeriClockControl(GPIOC,ENABLE);
	GPIO_Init(&GpioLed);

	while (1)
	{

		GPIO_ToggleOutputPin(GPIOC,GPIO_PIN_NO_9);

	}
	return 0;
}

void delay(int n) {
    int i;
    for (; n > 0; n--)
        for (i = 0; i < 5000; i++) ;
}
