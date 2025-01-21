/*
 * test.c
 *
 *  Created on: Nov 15, 2024
 *      Author: User
 */

#include <stdint.h>
#include "stm32l476xx.h"

//#define RCC_AHB2ENR  *((uint32_t*)0x4002104C)
//#define RCC_AHB2ENR_GPIOAEN 1<< 0

int main(void){

	RCC->AHB2ENR|=(1 << 0);
	GPIOA->MODER&=~(3<<10);
	GPIOA->MODER|=(1<<10);
	for(;;){

		GPIOA->ODR |= (1<<5);
		delay(1000);
		GPIOA->ODR &= ~(1<<5);
		delay(1000);
	}
}

void delay(int n) {
    int i;
    for (; n > 0; n--)
        for (i = 0; i < 500; i++) ;
}

