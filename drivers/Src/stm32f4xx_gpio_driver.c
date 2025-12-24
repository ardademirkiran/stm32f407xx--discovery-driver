/*
 * stm32f4xx_gpio_driver.c
 *
 *  Created on: Nov 29, 2025
 *      Author: ardademirkran
 */

#include "stm32f4xx_gpio_driver.h"

// API's supported by this driver


void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{

	uint32_t temp = 0;
	// Configure the mode of the GPIO pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp;
	} else {
		//interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {
			// falling edge trigger, Configure FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// clear the corresponding RTSR
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT) {
			// rising edge trigger, Configure RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// clear the corresponding FTSR
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT) {
			// Configure both FTSR and RTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}

		// Configure the GPIO Port selection with SYSCFG_EXTICR
		uint32_t syscfgRegisterSelector = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint32_t extiSelector = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

		uint32_t shift = extiSelector * 4;

		SYSCFG_PCLK_EN();

		// Clearing the 4 bit field
		SYSCFG->EXTICR[syscfgRegisterSelector] &= ~(0xF << shift);

		// Setting the port code
		SYSCFG->EXTICR[syscfgRegisterSelector] |= (portCode << shift);


		//Enable the exti interrup delivery using IMR - interrupt mask register
		EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	}


	// Configure the speed
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	// Configure the pupd settings
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	// Configure the output type register
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	// Configure the alternate function
	temp = 0;
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {
		uint8_t registerIndex = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		uint8_t pinIndex = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * pinIndex));
		pGPIOHandle->pGPIOx->AFR[registerIndex] &= ~( 0xF << (4 * pinIndex));
		pGPIOHandle->pGPIOx->AFR[registerIndex] |= temp;

	}
}

void GPIO_DeInit(GPIO_Handle_t *pGPIOHandle)
{
	if(pGPIOHandle->pGPIOx == GPIOA) {
				GPIOA_REG_RESET();
			} else if(pGPIOHandle->pGPIOx == GPIOB) {
				GPIOB_REG_RESET();
			} else if(pGPIOHandle->pGPIOx == GPIOB) {
				GPIOB_REG_RESET();
			} else if(pGPIOHandle->pGPIOx == GPIOC) {
				GPIOC_REG_RESET();
			} else if(pGPIOHandle->pGPIOx == GPIOD) {
				GPIOD_REG_RESET();
			} else if(pGPIOHandle->pGPIOx == GPIOE) {
				GPIOE_REG_RESET();
			} else if(pGPIOHandle->pGPIOx == GPIOF) {
				GPIOF_REG_RESET();
			} else if(pGPIOHandle->pGPIOx == GPIOG) {
				GPIOG_REG_RESET();
			} else if(pGPIOHandle->pGPIOx == GPIOH) {
				GPIOH_REG_RESET();
			} else if(pGPIOHandle->pGPIOx == GPIOI) {
				GPIOI_REG_RESET();
			} else if(pGPIOHandle->pGPIOx == GPIOJ) {
				GPIOJ_REG_RESET();
			} else if(pGPIOHandle->pGPIOx == GPIOK) {
				GPIOK_REG_RESET();
			}
}



uint8_t GPIO_ReadFromInputPin(GPIO_Handle_t *pGPIOx, uint8_t pinNumber)
{
	uint8_t value;
	value = (uint8_t) ((pGPIOx->pGPIOx->IDR >> pinNumber) & 0x00000001);

	return value;

}


uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;

	value = (uint16_t) (pGPIOx->IDR);

	return value;

}


void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value)
{

	if(value == GPIO_PIN_SET) {
		pGPIOx->ODR |= (1 << pinNumber);
	} else {
		pGPIOx->ODR |= ~(0 << pinNumber);
	}

}


void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value;
}


void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber)
{
	pGPIOx->ODR ^= (1 << pinNumber);
}


void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi) {
	if(EnOrDi == ENABLE) {
		if(IRQNumber <= 31) {
			//Configure NVIC_ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		} else if(IRQNumber < 64){
			//Confiugre NVIC_ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		} else if(IRQNumber < 96) {
			//Configure  NVIC_ISER2 register
			*NVIC_ISER3 |= (1 << (IRQNumber % 64));
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
			*NVIC_ICER3 |= (1 << (IRQNumber % 64));
		}
	}
}


void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
	uint8_t iprx = (uint8_t) (IRQNumber  / 4);
	uint8_t prix = IRQNumber % 4;
	uint8_t shift = (8 * prix) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_IPR_BASEADDR + (iprx)) |= (IRQPriority << shift);
}

void GPIO_IRQHandling(uint8_t pinNumber)
{
	//Clear the EXTI PR register, corresponding to the pin number
	if(EXTI->PR & (1 << pinNumber)) {
		EXTI->PR = (1 << pinNumber);
	}
}




void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE) {
		if(pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		} else if(pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		} else if(pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		} else if(pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		} else if(pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		} else if(pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		} else if(pGPIOx == GPIOF) {
			GPIOF_PCLK_EN();
		} else if(pGPIOx == GPIOG) {
			GPIOG_PCLK_EN();
		} else if(pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		} else if(pGPIOx == GPIOI) {
			GPIOI_PCLK_EN();
		} else if(pGPIOx == GPIOJ) {
			GPIOJ_PCLK_EN();
		} else if(pGPIOx == GPIOK) {
			GPIOK_PCLK_EN();
		}
	} else {
		if(pGPIOx == GPIOA) {
			GPIOA_PCLK_DI();
		} else if(pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		} else if(pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		} else if(pGPIOx == GPIOC) {
			GPIOC_PCLK_DI();
		} else if(pGPIOx == GPIOD) {
			GPIOD_PCLK_DI();
		} else if(pGPIOx == GPIOE) {
			GPIOE_PCLK_DI();
		} else if(pGPIOx == GPIOF) {
			GPIOF_PCLK_DI();
		} else if(pGPIOx == GPIOG) {
			GPIOG_PCLK_DI();
		} else if(pGPIOx == GPIOH) {
			GPIOH_PCLK_DI();
		} else if(pGPIOx == GPIOI) {
			GPIOI_PCLK_DI();
		} else if(pGPIOx == GPIOJ) {
			GPIOJ_PCLK_DI();
		} else if(pGPIOx == GPIOK) {
			GPIOK_PCLK_DI();
		}
	}
}


