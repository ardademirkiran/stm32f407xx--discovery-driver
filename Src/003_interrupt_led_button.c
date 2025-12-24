/*
 * main.c
 *
 *  Created on: Dec 24, 2025
 *      Author: ardademirkran
 */


#include "string.h"
#include "stm32f407xx.h"

void delay(void) {

	for (uint32_t i = 0; i < 500000/2; i++);
}

int main(void) {
	GPIO_Handle_t ledHandle;
	memset(&ledHandle, 0, sizeof(ledHandle));
	ledHandle.pGPIOx = GPIOD;
	ledHandle.GPIO_PinConfig.GPIO_PinNumber = 12;
	ledHandle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	ledHandle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	ledHandle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	ledHandle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GPIO_Handle_t buttonHandle;
	memset(&buttonHandle, 0, sizeof(buttonHandle));
	buttonHandle.pGPIOx = GPIOA;
	buttonHandle.GPIO_PinConfig.GPIO_PinNumber = 0;
	buttonHandle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	buttonHandle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	buttonHandle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	buttonHandle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_PeriClockControl(GPIOA, ENABLE);


	GPIO_Init(&ledHandle);
	GPIO_Init(&buttonHandle);

	GPIO_IRQInterruptConfig(EXTI0_IRQn, ENABLE);
	GPIO_IRQPriorityConfig(EXTI0_IRQn, 15);

	while(1);



	return 0;

}

void EXTI0_IRQHandler(void) {
	delay();
	GPIO_IRQHandling(0);
	GPIO_ToggleOutputPin(GPIOD, 12);
}
