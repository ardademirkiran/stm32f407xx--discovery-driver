/*
 * smt32f4xx_spi_driver.c
 *
 *  Created on: Dec 27, 2025
 *      Author: ardademirkran
 */


#include "stm32f4xx_spi_driver.h"


static void  spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void  spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void  spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

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


void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len) {
	while(len > 0) {
		while(!(pSPIx->SPI_SR & (1 << 0)));

		if(pSPIx->SPI_CR1 & (1 << 11)){
			*((uint16_t*)pRxBuffer) = pSPIx->SPI_DR;
			len -= 2;
			pRxBuffer += 2;
		} else {
			*pRxBuffer = pSPIx->SPI_DR;
			len--;
			pRxBuffer++;
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


void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi) {
	if(EnOrDi == ENABLE) {
		pSPIx->SPI_CR2 |= (1 << 2);
	} else {
		pSPIx->SPI_CR2 &= ~(1 << 2);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE) {
		pSPIx->SPI_CR1 |= (1 << 8);
	} else {
		pSPIx->SPI_CR1 &= ~(1 << 8);
	}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flagIndex){
	if (pSPIx->SPI_SR & flagIndex){
		return 1;
	} else {
		return 0;
	}
}

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi){
	if(EnOrDi == ENABLE) {
		if(IRQNumber <= 31) {
			//Configure NVIC_ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		} else if(IRQNumber < 64){
			//Confiugre NVIC_ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		} else if(IRQNumber < 96) {
			//Configure  NVIC_ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	} else {
		if(IRQNumber <= 31) {
			//Configure NVIC_ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		} else if(IRQNumber < 64){
			//Confiugre NVIC_ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		} else if(IRQNumber < 96) {
			//Configure  NVIC_ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	uint8_t iprx = (uint8_t) (IRQNumber  / 4);
	uint8_t prix = IRQNumber % 4;
	uint8_t shift = (8 * prix) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_IPR_BASEADDR + (iprx)) |= (IRQPriority << shift);
}

void SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len){
	if(pSPIHandle->txState != 2) {
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->txLen = len;
		pSPIHandle->txState = 2;
		pSPIHandle->pSPIx->SPI_CR2 |= (1 << 7);
	}
}
void SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len){
	if(pSPIHandle->rxState != 2) {
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->rxLen = len;
		pSPIHandle->rxState = 2;
		pSPIHandle->pSPIx->SPI_CR2 |= (1 << 6);
	}
}

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle){
 //first check for TXE
	uint8_t txeCheck, txeieCheck;
	txeCheck = pSPIHandle->pSPIx->SPI_SR & (1 << 1);
	txeieCheck = pSPIHandle->pSPIx->SPI_CR2 & (1 << 7);

	if(txeCheck && txeieCheck) {
		//handle txe interrupt
		spi_txe_interrupt_handle(pSPIHandle);
	}

	uint8_t rxneCheck, rxneieCheck;
	rxneCheck = pSPIHandle->pSPIx->SPI_SR & (1 << 0);
	rxneieCheck = pSPIHandle->pSPIx->SPI_CR2 & (1 << 6);

	if(rxneCheck && rxneieCheck){
		//handle rxne interrupt
		spi_rxne_interrupt_handle(pSPIHandle);
	}

	uint8_t ovrCheck, errieCheck;

	ovrCheck = pSPIHandle->pSPIx->SPI_SR & (1 << 6);
	errieCheck = pSPIHandle->pSPIx->SPI_CR2 & (1 << 5);

	if(ovrCheck && errieCheck) {
		spi_ovr_err_interrupt_handle(pSPIHandle);
	}



}


static void  spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle){
	if(pSPIHandle->pSPIx->SPI_CR1 & (1 << 11)){
		pSPIHandle->pSPIx->SPI_DR = *((uint16_t*) pSPIHandle->pTxBuffer);
		pSPIHandle->txLen -= 2;
		pSPIHandle->pTxBuffer += 2;
	} else {
		pSPIHandle->pSPIx->SPI_DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->txLen--;
		pSPIHandle->pTxBuffer++;
	}

	if(pSPIHandle->txLen == 0) {
		pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << 7);
		pSPIHandle->pTxBuffer = NULL;
		pSPIHandle->txLen = 0;
		pSPIHandle->txState = 0;
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}
static void  spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle){
	if(pSPIHandle->pSPIx->SPI_CR1 & (1 << 11)){
		*((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->SPI_DR;
		pSPIHandle->rxLen -= 2;
		pSPIHandle->pRxBuffer += 2;
	} else {
		*(pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->SPI_DR;
		pSPIHandle->rxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if(pSPIHandle->rxLen == 0) {
		pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << 6);
		pSPIHandle->pRxBuffer = NULL;
		pSPIHandle->rxLen = 0;
		pSPIHandle->rxState = 0;
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}


static void  spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{

	uint8_t temp;
	if(pSPIHandle->txState != 2)
	{
		temp = pSPIHandle->pSPIx->SPI_DR;
		temp = pSPIHandle->pSPIx->SPI_SR;
	}
	(void)temp;

	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);

}


void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->SPI_CR2 &= ~( 1 << 7);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->txLen = 0;
	pSPIHandle->txState = 0;

}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->SPI_CR2 &= ~( 1 << 6);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->rxLen = 0;
	pSPIHandle->rxState = 0;

}



void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->SPI_DR;
	temp = pSPIx->SPI_SR;
	(void)temp;

}


__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{

	//This is a weak implementation . the user application may override this function.
}

