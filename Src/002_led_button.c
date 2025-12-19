/*
 * 002_led_button.c
 *
 *  Created on: Dec 18, 2025
 *      Author: ardademirkran
 */

#include "stm32f407xx.h"

void delay(void) {

	for (uint32_t i = 0; i < 500000/2; i++);
}

int main(void) {
	GPIO_Handle_t ledHandle;
	ledHandle.pGPIOx = GPIOD;
	ledHandle.GPIO_PinConfig.GPIO_PinNumber = 12;
	ledHandle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	ledHandle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	ledHandle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;


	GPIO_Handle_t buttonHandle;
	buttonHandle.pGPIOx = GPIOA;
	buttonHandle.GPIO_PinConfig.GPIO_PinNumber = 0;
	buttonHandle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	buttonHandle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	buttonHandle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_PeriClockControl(GPIOA, ENABLE);


	GPIO_Init(&ledHandle);
	GPIO_Init(&buttonHandle);

	while(1) {
		if(GPIO_ReadFromInputPin(&buttonHandle, 0) == 1){
			delay();
			GPIO_ToggleOutputPin(GPIOD, ledHandle.GPIO_PinConfig.GPIO_PinNumber);
		}
	}



	return 0;

}
