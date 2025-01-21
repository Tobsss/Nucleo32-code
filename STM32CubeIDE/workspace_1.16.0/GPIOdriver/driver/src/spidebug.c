/*
 * spidebug.c
 *
 *  Created on: Dec 3, 2024
 *      Author: User
 */

#include "stm32l476xx.h"

void initSPI(void);
void initClocks(void);
void configSPI2pins(void);
void setPinMode(void);
void setAF(void);
void configSPI(void);
uint8_t transferSPI(uint8_t tx_data);

void initClocks(void)
{
	RCC->AHB2ENR |= ((1<<1)| (1<<0));
	RCC->APB1ENR1 |= (1<<14);
}

void setPinMode(void)
{
	GPIOB->MODER &= ~((3<< 2*12)| (3<< 2*13) | (3<< 2*15));
	GPIOB->MODER |= ((2<< 2*12)| (2<< 2*13) | (2<< 2*15));
}

void setAF(void)
{
	GPIOB->AFRH |= ((1<<30)|(1<<28)|(1<<22)|(1<<20)|(1<<18)|(1<<16));
}
void configSPI(void)
{
	SPI2->CR1 &= ~((1<<15)| (1<<13)| (1<<10)|(1<<9)|(1<<7)|(7<<3));
	SPI2->CR1 |= ((5<<3)|(1<<2)|(1<<1)|(1<<0));

	SPI2->CR2 &= ~((1<<12)| (7<<5)| (1<<4)|(1<<3)|(3<<0));
	SPI2->CR2 |= ((15<<8)|(1<<2));
}
