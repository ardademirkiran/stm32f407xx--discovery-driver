/*
 * 001_led_toogle.c
 *
 *  Created on: Dec 18, 2025
 *      Author: ardademirkran
 */


#include "stm32f407xx.h"

void delay(void) {

	for (uint32_t i = 0; i < 500000; i++);
}

int main(void) {
	GPIO_Handle_t ledHandle;
	ledHandle.pGPIOx = GPIOD;
	ledHandle.GPIO_PinConfig.GPIO_PinNumber = 12;
	ledHandle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	ledHandle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	ledHandle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	ledHandle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	RCC->AHB1ENR |= (1 << 3);
	GPIO_Init(&ledHandle);

	while(1) {
		GPIO_ToggleOutputPin(GPIOD, ledHandle.GPIO_PinConfig.GPIO_PinNumber);
		delay();
	}

	return 0;
}
