/*
 * 005_spi_comm_to_arduino.c
 *
 *  Created on: Dec 27, 2025
 *      Author: ardademirkran
 */

#include "stm32f407xx.h"
#include "stm32f4xx_spi_driver.h"
#include "string.h"

#define CS_PIN		12
#define SCLK_PIN	13
#define MISO_PIN	14
#define MOSI_PIN	15

/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 -> SPI2_SCLK
 * ALT function mode : 5
 */

SPI_Handle_t SPI2Handle;

void SPI2_CS_GPIOInit(void)
{
    GPIO_Handle_t gpioCS;

    GPIO_PeriClockControl(GPIOB, ENABLE);

    gpioCS.pGPIOx = GPIOB;
    gpioCS.GPIO_PinConfig.GPIO_PinNumber = CS_PIN;
    gpioCS.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    gpioCS.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    gpioCS.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    gpioCS.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_Init(&gpioCS);

	GPIO_WriteToOutputPin(GPIOB, CS_PIN, GPIO_PIN_SET);

}


void SPI2_GPIOInits(void){
	GPIO_PeriClockControl(GPIOB, ENABLE);

	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;


	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = SCLK_PIN;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = MISO_PIN;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = MOSI_PIN;
	GPIO_Init(&SPIPins);
}

void SPI2_Inits(void){
	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SCLKSpeed = SPI_SCLK_SPEED_DIV32;
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8_BIT;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_ENABLED;

	SPI_PeriClockControl(&SPI2Handle, ENABLE);
	SPI_Init(&SPI2Handle);
}

int main(void) {
	uint8_t userData[] = "Hello ESP!";
	SPI2_GPIOInits();
	SPI2_Inits();
	SPI_IRQInterruptConfig(SPI2_IRQn, ENABLE);
	SPI_IRQPriorityConfig(SPI2_IRQn, 15);

	SPI2_CS_GPIOInit();
	GPIO_WriteToOutputPin(GPIOB, CS_PIN, GPIO_PIN_RESET);

	SPI_PeripheralControl(SPI2, ENABLE);
	for(int i = 0; i < 5000; i++);
	SPI_SendDataIT(&SPI2Handle, (uint8_t*)userData, strlen((char*)userData));

	while(1) {

	}
	return 0;
}

void SPI2_IRQHandler(void) {
	SPI_IRQHandling(&SPI2Handle);
}

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
    if (AppEv == SPI_EVENT_TX_CMPLT)
    {
        while (pSPIHandle->pSPIx->SPI_SR & (1 << SPI_SR_BSY_POS));

        GPIO_WriteToOutputPin(GPIOB, CS_PIN, GPIO_PIN_SET);

        SPI_PeripheralControl(pSPIHandle->pSPIx, DISABLE);
    }
}


