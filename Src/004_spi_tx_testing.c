/*
 * 004_spi_tx_testing.c
 *
 *  Created on: Dec 27, 2025
 *      Author: ardademirkran
 */

#include "stm32f407xx.h"
#include "stm32f4xx_spi_driver.h"
#include "string.h"

/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 -> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * ALT function mode : 5
 */


void SPI2_GPIOInits(void){
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;


	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = 13;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = 14;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = 15;
	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = 12;
	GPIO_Init(&SPIPins);
}

void SPI2_Inits(void){

	SPI_Handle_t SPI2Handle;
	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SCLKSpeed = SPI_SCLK_SPEED_DIV2;
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8_BIT;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_ENABLED;


	SPI_Init(&SPI2Handle);
}
int main(void) {

	char userData[]  = "Hello World";

	SPI2_GPIOInits();
	SPI2_Inits();

	SPI_SendData(SPI2, (uint8_t*)userData, strlen(userData));

	// This must be done last
	SPI_PeripheralControl(SPI2, ENABLE);
	return 0;
}

