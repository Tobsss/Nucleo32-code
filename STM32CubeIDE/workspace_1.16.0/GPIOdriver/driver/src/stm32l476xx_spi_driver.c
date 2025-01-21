/*
 * stm32l476xx_spi_driver.c
 *
 *  Created on: Nov 22, 2024
 *      Author: User
 */


#include "stm32l476xx_spi_driver.h"
#include "stm32l476xx.h"

/**************************************************
 * @fn				- SPI_PeriClockControl
 *
 * @brief			- This function enables or disables peripheral clock for given SPI Peripheral
 *
 * @param[in]		- base address of the spi peripheral
 * @param[in]		- ENABLE or DISABLE macros
 * @param[in]
 *
 * @return			- none
 *
 * @Note			- none
 *
 *
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK;
			}else if (pSPIx == SPI2)
			{
				SPI2_PCLK;
			}else if (pSPIx == SPI3)
			{
				SPI3_PCLK;
			}
		}
		else
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_DI;
			}else if (pSPIx == SPI2)
			{
				SPI2_PCLK_DI;
			}else if (pSPIx == SPI3)
			{
				SPI3_PCLK_DI;
			}
		}

}

/**************************************************
 * @fn				- SPI_Init
 *
 * @brief			- This function initializes the SPI peripheral
 *
 * @param[in]		- SPI Handle struct
 * @param[in]		-
 * @param[in]
 *
 * @return			- none
 *o
 * @Note			- none
 *
 *l
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//PERIPHERAL CLOCK ENABLE
	uint32_t tempreg = 0;

	//first let's configure the SPI_CR1 register


	//1. configure the device mode
	tempreg |= pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR;

	//2. CONFIGURE THE BUS
	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//Bidi mode should be cleared
		tempreg &= ~(1<<SPI_CR1_BIDIMODE);
	}else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{	//bidi mode should be set
		tempreg |= (1<<SPI_CR1_BIDIMODE);

	}else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//bidi mode should be cleared
		tempreg &= ~(1<<SPI_CR1_BIDIMODE);
		//RX only bit must be set
		tempreg |= (1<<SPI_CR1_RX);

	}

	//3. Configure the spi serial clock speed(baud rate)
	tempreg |= pSPIHandle->SPI_Config.SPI_SclkSpeed <<SPI_CR1_BR;

	//configure the SSM
	tempreg|= pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM;


	//4. Configure the DFF
	tempreg |= pSPIHandle->SPI_Config.SPI_DFF<<SPI_CR1_DFF ;

	//5. Configure the CPOL
	tempreg |= pSPIHandle->SPI_Config.SPI_CPOL<<SPI_CR1_CPOL;

	//5. Configure the CPHA
	tempreg |= pSPIHandle->SPI_Config.SPI_CPHA<< SPI_CR1_CPHA;

	tempreg |= 1<<SPI_CR1_LSB;

	pSPIHandle->pSPIx->CR1 = tempreg;

	pSPIHandle->pSPIx->CR2 &= ~(0xF<<SPI_CR2_DS);
	pSPIHandle->pSPIx->CR2 |= (9<<SPI_CR2_DS);

}  //I have not written the de-initialization

uint32_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName) // kinda confusing
{
    if(pSPIx-> SR & FlagName)
    {
    	return FLAG_SET;
    }
	return FLAG_RESET;
}


/**************************************************
 * @fn				- SPI_SendData
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]
 *
 * @return			-
 *
 * @Note			- This is a blocking call
 *
 *
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	pSPIx->DR = 0;
	while(Len>0)
	{

		//1. wait for TXE to be set
		//while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG)  == FLAG_RESET);
	    while (!(pSPIx->SR & (SPI_TXE_FLAG)));
	    while ((pSPIx->SR & SPI_SR_BSY));
		//2. check the DFF bit in CR1
		if((pSPIx-> CR1 & (1 << SPI_CR1_DFF)))
		{
			//16 BIT DFF
			//1. load the data in to the DR

			pSPIx->DR += *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}else
		{
			//8 bit DFF
			//1. load the data in to the DR
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}
/*
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
    while (Len > 0)
    {
        // 1. Wait for TXE to be set (Transmit Buffer Empty)
        while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

        // 2. Check the DFF (Data Frame Format) bit in CR1
        if (pSPIx->CR1 & (1 << SPI_CR1_DFF))
        {
            // 16-bit DFF
            // 1. Load the data into the DR register (cast to 16-bit)
            pSPIx->DR |= *((uint16_t*)pTxBuffer)<<8;
            Len -= 2;  // Two bytes sent in 16-bit mode
            pTxBuffer += 2;  // Move the pointer by 2 bytes
        }
        else
        {
            // 8-bit DFF
            // 1. Load the data into the DR register
            pSPIx->DR = *pTxBuffer;
            Len--;  // One byte sent in 8-bit mode
            pTxBuffer++;  // Move the pointer by 1 byte
        }
    }
}*/
/**************************************************
 * @fn				- SPI_ReceiveData
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]
 *
 * @return			-
 *
 * @Note			- This is a blocking call
 *
 *
 */

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len>0)
	{	 *pRxBuffer = 0;
		 pSPIx->DR = 0XFF;
		//1. wait for RXNE to be set
		//while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG)  == FLAG_RESET);
		//while (!(pSPIx->SR & (1<< 1)));
		 while ((pSPIx->SR & SPI_SR_BSY));
		 while ((pSPIx->SR & (SPI_RXNE_FLAG)));

		//2. check the DFF bit in CR1
		if((pSPIx-> CR1 & (1 << SPI_CR1_DFF)))
		{
			//16 BIT DFF
			//1. load the data From DR to Rx buffer address
			*((uint16_t*)pRxBuffer) = pSPIx->DR ;
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++;
		}else
		{
			//8 bit DFF
			//1. load the data in to the DR
			*(pRxBuffer) = pSPIx->DR ;
			Len--;
			pRxBuffer++;
		}
	}
}
/**************************************************
 * @fn				- SPI_PeripheralControl
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]
 *
 * @return			-
 *
 * @Note			-
 *
 *
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}


void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
		{
			pSPIx->CR1 |= (1 << SPI_CR1_SSI);
		}else
		{
			pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
		}
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
			{
				pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
			}else
			{
				pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
			}
}


