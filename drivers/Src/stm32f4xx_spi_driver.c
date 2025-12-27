/*
 * smt32f4xx_spi_driver.c
 *
 *  Created on: Dec 27, 2025
 *      Author: ardademirkran
 */


#include "stm32f4xx_spi_driver.h"

void SPI_PeriClockControl(SPI_Handle_t *pSPIHandle, uint8_t EnOrDi){
	if(EnOrDi == ENABLE) {
		if(pSPIHandle->pSPIx == SPI1) {
			SPI1_PCLK_EN();
		} else if(pSPIHandle->pSPIx == SPI2) {
			SPI2_PCLK_EN();
		} else if(pSPIHandle->pSPIx == SPI3) {
			SPI3_PCLK_EN();
		}
	} else {
		if(pSPIHandle->pSPIx == SPI1) {
			SPI1_PCLK_DI();
		} else if(pSPIHandle->pSPIx == SPI2) {
			SPI2_PCLK_DI();
		} else if(pSPIHandle->pSPIx == SPI3) {
			SPI3_PCLK_DI();
		}
	}
}

void SPI_Init(SPI_Handle_t *pSPIHandle){

	SPI_PeriClockControl(pSPIHandle, ENABLE);

	uint32_t temp = 0;

	// Configure the device mode
	temp |= (pSPIHandle->SPIConfig.SPI_DeviceMode << 2);

	// Configure the bus config

	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		temp &= ~(1 << 15);
	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		temp |= (1 << 15);
	} else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY){
		temp &= ~(1 << 15);
		temp |= (1 << 10);
	}

	temp |= (pSPIHandle->SPIConfig.SPI_SCLKSpeed << 3);
	temp |= (pSPIHandle->SPIConfig.SPI_DFF << 11);
	temp |= (pSPIHandle->SPIConfig.SPI_CPHA << 0);
	temp |= (pSPIHandle->SPIConfig.SPI_CPOL << 1);
	temp |= (pSPIHandle->SPIConfig.SPI_SSM << 9);
	temp |= (1 << 8);
	pSPIHandle->pSPIx->SPI_CR1 = temp;
}


void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len) {

	while (len > 0) {

		//wait until TXE is set in the SPI_SR
		while(!(pSPIx->SPI_SR & (1 << 1)));

		if(pSPIx->SPI_CR1 & (1 << 11)){
			pSPIx->SPI_DR = *((uint16_t*) pTxBuffer);
			len -= 2;
			pTxBuffer += 2;
		} else {
			pSPIx->SPI_DR = *pTxBuffer;
			len--;
			pTxBuffer++;
		}
	}
}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE) {
		pSPIx->SPI_CR1 |= (1 << 6);
	} else {
		pSPIx->SPI_CR1 &= ~(1 << 6);
	}
}
