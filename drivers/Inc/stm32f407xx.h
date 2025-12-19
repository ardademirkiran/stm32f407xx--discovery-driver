/*
 * stm32f407xx.h
 *
 *  Created on: Nov 29, 2025
 *      Author: ardademirkran
 */
#include <stdint.h>


#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

/*
 * Base addresses of Flash and SRAM memories
 */

#define FLASH_BASE_ADDR 		0x08000000U
#define SRAM1_BASE_ADDR 		0x20000000U //112 KB
#define SRAM2_BASE_ADDR			0x2001C000U
#define ROM_MEMORY_BASE_ADDR 	0x1FFF0000U

#define AHB1_BASE_ADDR 			0x40020000U
#define AHB2_BASE_ADDR 			0x50000000U
#define AHB3_BASE_ADDR			0xA0000000U

#define APB1_BASE_ADDR			0x40000000U
#define APB2_BASE_ADDR			0x40010000U


/*
 * AHB_1 Peripherals
 */
#define GPIO_A_BASE_ADDR		(AHB1_BASE_ADDR + 0x0000)
#define GPIO_B_BASE_ADDR		(AHB1_BASE_ADDR + 0x0400)
#define GPIO_C_BASE_ADDR		(AHB1_BASE_ADDR + 0x0800)
#define GPIO_D_BASE_ADDR		(AHB1_BASE_ADDR + 0x0C00)
#define GPIO_E_BASE_ADDR		(AHB1_BASE_ADDR + 0x1000)
#define GPIO_F_BASE_ADDR		(AHB1_BASE_ADDR + 0x1400)
#define GPIO_G_BASE_ADDR		(AHB1_BASE_ADDR + 0x1800)
#define GPIO_H_BASE_ADDR		(AHB1_BASE_ADDR + 0x1C00)
#define GPIO_I_BASE_ADDR		(AHB1_BASE_ADDR + 0x2000)
#define GPIO_J_BASE_ADDR		(AHB1_BASE_ADDR + 0x2400)
#define GPIO_K_BASE_ADDR		(AHB1_BASE_ADDR + 0x2800)

/*
 * APB1 Peripherals
 */

#define I2C1_BASE_ADDR			(APB1_BASE_ADDR + 0x5400)
#define I2C2_BASE_ADDR			(APB1_BASE_ADDR + 0x5800)
#define I2C3_BASE_ADDR			(APB1_BASE_ADDR + 0x5C00)

#define SPI2_BASE_ADDR			(APB1_BASE_ADDR + 0x3800)
#define SPI3_BASE_ADDR			(APB1_BASE_ADDR + 0x3C00)

#define USART2_BASE_ADDR		(APB1_BASE_ADDR + 0x4400)
#define USART3_BASE_ADDR		(APB1_BASE_ADDR + 0x4800)
#define UART4_BASE_ADDR			(APB1_BASE_ADDR + 0x4C00)
#define UART5_BASE_ADDR			(APB1_BASE_ADDR + 0x5000)


/*
 * APB2 Peripherals
 */

#define EXTI_BASE_ADDR			(APB2_BASE_ADDR + 0x3C00)
#define SPI1_BASE_ADDR			(APB2_BASE_ADDR + 0x3000)
#define SYSCFG_BASE_ADDR		(APB2_BASE_ADDR + 0x3800)
#define USART1_BASE_ADDR		(APB2_BASE_ADDR + 0x1000)
#define USART6_BASE_ADDR		(APB2_BASE_ADDR + 0x1400)


/*
 * SPI1 Register Addresses
 */

#define SPI1_CR1_ADDR 			(SPI1_BASE_ADDR + 0x0000)
#define SPI1_CR2_ADDR 			(SPI1_BASE_ADDR + 0x0004)
#define SPI1_SR_ADDR 			(SPI1_BASE_ADDR + 0x0008)
#define SPI1_DR_ADDR 			(SPI1_BASE_ADDR + 0x000C)
#define SPI1_CRCPR_ADDR 		(SPI1_BASE_ADDR + 0x0010)
#define SPI1_RXCRC_ADDR 		(SPI1_BASE_ADDR + 0x0014)
#define SPI1_TXCRC_ADDR 		(SPI1_BASE_ADDR + 0x0018)
#define SPI1_I2SCFGR			(SPI1_BASE_ADDR + 0x001C)
#define SPI1_I2SPR 				(SPI1_BASE_ADDR + 0x0020)



#define SRAM 					SRAM1_BASE_ADDR


typedef struct
{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2];
}GPIO_RegDef_t;

#define GPIOA		((GPIO_RegDef_t*) GPIO_A_BASE_ADDR)
#define GPIOB		((GPIO_RegDef_t*) GPIO_B_BASE_ADDR)
#define GPIOC		((GPIO_RegDef_t*) GPIO_C_BASE_ADDR)
#define GPIOD		((GPIO_RegDef_t*) GPIO_D_BASE_ADDR)
#define GPIOE		((GPIO_RegDef_t*) GPIO_E_BASE_ADDR)
#define GPIOF		((GPIO_RegDef_t*) GPIO_F_BASE_ADDR)
#define GPIOG		((GPIO_RegDef_t*) GPIO_G_BASE_ADDR)
#define GPIOH		((GPIO_RegDef_t*) GPIO_H_BASE_ADDR)
#define GPIOI		((GPIO_RegDef_t*) GPIO_I_BASE_ADDR)
#define GPIOJ		((GPIO_RegDef_t*) GPIO_J_BASE_ADDR)
#define GPIOK		((GPIO_RegDef_t*) GPIO_K_BASE_ADDR)







#define RCC_BASE_ADDR			0x40023800U



typedef struct {
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	volatile uint32_t RESERVED0;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR ;
	volatile uint32_t RESERVED1;
	volatile uint32_t RESERVED2;
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	volatile uint32_t RESERVED3;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	volatile uint32_t RESERVED4;
	volatile uint32_t RESERVED5;
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	volatile uint32_t RESERVED6;
	volatile uint32_t RESERVED7;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	volatile uint32_t RESERVED8;
	volatile uint32_t RESERVED9;
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
} RCC_RegDef_t;

#define RCC		((RCC_RegDef_t*) RCC_BASE_ADDR)

/*
 * Clock enable for GPIOx
 */

#define GPIOA_PCLK_EN() ( RCC->AHB1ENR |= ( 1 << 0 ) )
#define GPIOB_PCLK_EN() ( RCC->AHB1ENR |= ( 1 << 1 ) )
#define GPIOC_PCLK_EN() ( RCC->AHB1ENR |= ( 1 << 2 ) )
#define GPIOD_PCLK_EN() ( RCC->AHB1ENR |= ( 1 << 3 ) )
#define GPIOE_PCLK_EN() ( RCC->AHB1ENR |= ( 1 << 4 ) )
#define GPIOF_PCLK_EN() ( RCC->AHB1ENR |= ( 1 << 5 ) )
#define GPIOG_PCLK_EN() ( RCC->AHB1ENR |= ( 1 << 6 ) )
#define GPIOH_PCLK_EN() ( RCC->AHB1ENR |= ( 1 << 7 ) )
#define GPIOI_PCLK_EN() ( RCC->AHB1ENR |= ( 1 << 8 ) )
#define GPIOJ_PCLK_EN() ( RCC->AHB1ENR |= ( 1 << 9 ) )
#define GPIOK_PCLK_EN() ( RCC->AHB1ENR |= ( 1 << 10 ) )

/*
 * Clock enable for I2Cx
 */
#define I2C1_PCLK_EN() ( RCC->APB1ENR |= ( 1 << 21 ) )
#define I2C2_PCLK_EN() ( RCC->APB1ENR |= ( 1 << 22 ) )
#define I2C3_PCLK_EN() ( RCC->APB1ENR |= ( 1 << 23 ) )

/*
 * Clock enable for USARTx
 */
#define USART2_PCLK_EN() ( RCC->APB1ENR |= ( 1 << 17 ) )
#define USART3_PCLK_EN() ( RCC->APB1ENR |= ( 1 << 18 ) )
#define UART4_PCLK_EN()	 ( RCC->APB1ENR |= ( 1 << 19 ) )
#define UART5_PCLK_EN()	 ( RCC->APB1ENR |= ( 1 << 20 ) )


/*
 * Clock enable for SPIx
 */
#define SPI1_PCLK_EN() ( RCC->APB2ENR |= ( 1 << 12 ) )


/*
 * Clock disable for GPIOx
 */
#define GPIOA_PCLK_DI() ( RCC->AHB1ENR &= ~( 1 << 0 ) )
#define GPIOB_PCLK_DI() ( RCC->AHB1ENR &= ~( 1 << 1 ) )
#define GPIOC_PCLK_DI() ( RCC->AHB1ENR &= ~( 1 << 2 ) )
#define GPIOD_PCLK_DI() ( RCC->AHB1ENR &= ~( 1 << 3 ) )
#define GPIOE_PCLK_DI() ( RCC->AHB1ENR &= ~( 1 << 4 ) )
#define GPIOF_PCLK_DI() ( RCC->AHB1ENR &= ~( 1 << 5 ) )
#define GPIOG_PCLK_DI() ( RCC->AHB1ENR &= ~( 1 << 6 ) )
#define GPIOH_PCLK_DI() ( RCC->AHB1ENR &= ~( 1 << 7 ) )
#define GPIOI_PCLK_DI() ( RCC->AHB1ENR &= ~( 1 << 8 ) )
#define GPIOJ_PCLK_DI() ( RCC->AHB1ENR &= ~( 1 << 9 ) )
#define GPIOK_PCLK_DI() ( RCC->AHB1ENR &= ~( 1 << 10 ) )

/*
 * Clock disable for I2Cx
 */
#define I2C1_PCLK_DI() ( RCC->APB1ENR &= ~( 1 << 21 ) )
#define I2C2_PCLK_DI() ( RCC->APB1ENR &= ~( 1 << 22 ) )
#define I2C3_PCLK_DI() ( RCC->APB1ENR &= ~( 1 << 23 ) )


/*
 * Clock disable for USARTx
 */
#define USART2_PCLK_DI() ( RCC->APB1ENR &= ~( 1 << 17 ) )
#define USART3_PCLK_DI() ( RCC->APB1ENR &= ~( 1 << 18 ) )
#define UART4_PCLK_DI()	 (RCC->APB1ENR &= ~( 1 << 19 ) )
#define UART5_PCLK_DI()	 (RCC->APB1ENR &= ~( 1 << 20 ) )

/*
 * Clock disable for SPIx
 */
#define SPI1_PCLK_DI() ( RCC->APB2ENR &= ~( 1 << 12 ) )



//generic macros

#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE
#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET RESET





/*
 * GPIO register reset macro
 */

#define GPIOA_REG_RESET()	do{( RCC->AHB1RSTR |= ( 1 << 0 ) ); ( RCC->AHB1RSTR &= ~( 1 << 0 ) );} while(0)
#define GPIOB_REG_RESET()	do{( RCC->AHB1RSTR |= ( 1 << 1 ) ); ( RCC->AHB1RSTR &= ~( 1 << 1 ) );} while(0)
#define GPIOC_REG_RESET()	do{( RCC->AHB1RSTR |= ( 1 << 2 ) ); ( RCC->AHB1RSTR &= ~( 1 << 2 ) );} while(0)
#define GPIOD_REG_RESET()	do{( RCC->AHB1RSTR |= ( 1 << 3 ) ); ( RCC->AHB1RSTR &= ~( 1 << 3 ) );} while(0)
#define GPIOE_REG_RESET()	do{( RCC->AHB1RSTR |= ( 1 << 4 ) ); ( RCC->AHB1RSTR &= ~( 1 << 4 ) );} while(0)
#define GPIOF_REG_RESET()	do{( RCC->AHB1RSTR |= ( 1 << 5 ) ); ( RCC->AHB1RSTR &= ~( 1 << 5 ) );} while(0)
#define GPIOG_REG_RESET()	do{( RCC->AHB1RSTR |= ( 1 << 6 ) ); ( RCC->AHB1RSTR &= ~( 1 << 6 ) );} while(0)
#define GPIOH_REG_RESET()	do{( RCC->AHB1RSTR |= ( 1 << 7 ) ); ( RCC->AHB1RSTR &= ~( 1 << 7 ) );} while(0)
#define GPIOI_REG_RESET()	do{( RCC->AHB1RSTR |= ( 1 << 8 ) ); ( RCC->AHB1RSTR &= ~( 1 << 8 ) );} while(0)
#define GPIOJ_REG_RESET()	do{( RCC->AHB1RSTR |= ( 1 << 9 ) ); ( RCC->AHB1RSTR &= ~( 1 << 9 ) );} while(0)
#define GPIOK_REG_RESET()	do{( RCC->AHB1RSTR |= ( 1 << 10 ) ); ( RCC->AHB1RSTR &= ~( 1 << 10 ) );} while(0)














#include "stm32f4xx_gpio_driver.h"




#endif /* INC_STM32F407XX_H_ */
