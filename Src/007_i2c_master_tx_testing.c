/*
 * 007_i2c_master_tx_testing.c
 *
 *  Created on: Jan 24, 2026
 *      Author: ardademirkran
 */


#include "stm32f407xx.h"
#include "stm32f407x_i2c_driver.h"
#include "string.h"

#define SCL_PIN		6
#define SDA_PIN		9
#define BTN_PIN		0

I2C_Handle_t I2C1_handle;

uint8_t some_data[] = "Testing I2C master tx.";

void delay(void) {

	for (uint32_t i = 0; i < 500000/2; i++);
}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn;

	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = BTN_PIN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);

}




void I2C1_GPIOInits(void) {
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;


	//SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = SCL_PIN;

	GPIO_Init(&I2CPins);

	I2CPins.GPIO_PinConfig.GPIO_PinNumber = SDA_PIN;

	GPIO_Init(&I2CPins);

}

void I2C1_Inits(void) {
	I2C1_handle.pI2Cx = I2C1;
	I2C1_handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C1_handle.I2C_Config.I2C_DeviceAddress = 0x00; // change it to esps address later
	I2C1_handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1_handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1_handle);
}

int main(void) {
	I2C1_GPIOInits();

	I2C1_Inits();

	I2C_PeripheralControl(I2C1_handle.pI2Cx, ENABLE);

	while(1)
	{
		while( ! GPIO_ReadFromInputPin(GPIOA, BTN_PIN));

		delay();

		I2C_MasterSendData(&I2C1_handle, some_data, strlen((char*) some_data), 0x00, 0);
	}

}
