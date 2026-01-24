/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Jan 24, 2026
 *      Author: ardademirkran
 */


#include "stm32f407x_i2c_driver.h"

uint8_t APB1_PreScaler[4] = { 2, 4 , 8, 16};
uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};

uint32_t  RCC_GetPLLOutputClock()
{

	return 0;
}

uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1,SystemClk;

	uint8_t clksrc,temp,ahbp,apb1p;

	clksrc = ((RCC->CFGR >> 2) & 0x3);

	if(clksrc == 0 )
	{
		SystemClk = 16000000;
	}else if(clksrc == 1)
	{
		SystemClk = 8000000;
	}else if (clksrc == 2)
	{
		SystemClk = RCC_GetPLLOutputClock();
	}

	//for ahb
	temp = ((RCC->CFGR >> 4 ) & 0xF);

	if(temp < 8)
	{
		ahbp = 1;
	}else
	{
		ahbp = AHB_PreScaler[temp-8];
	}



	//apb1
	temp = ((RCC->CFGR >> 10 ) & 0x7);

	if(temp < 4)
	{
		apb1p = 1;
	}else
	{
		apb1p = APB1_PreScaler[temp-4];
	}

	pclk1 =  (SystemClk / ahbp) /apb1p;

	return pclk1;
}

void I2C_Init(I2C_Handle_t *pI2CHandle){

	uint32_t pclk1 = RCC_GetPCLK1Value();
	uint32_t freq = pclk1 / 1000000;
	uint32_t temp_register = 0;

	I2C_PeriClockControl(pI2CHandle, ENABLE);

	temp_register = pI2CHandle->I2C_Config.I2C_AckControl;
	pI2CHandle->pI2Cx->CR1 |= (temp_register << CR1_BIT_ACK);

	pI2CHandle->pI2Cx->CR2 &= ~(0x3F << CR2_BIT_FREQ);
	pI2CHandle->pI2Cx->CR2 |= (freq << CR2_BIT_FREQ);

	temp_register = pI2CHandle->I2C_Config.I2C_DeviceAddress;
	pI2CHandle->pI2Cx->OAR1 &= ~(0x7F << OAR1_BIT_ADD);
	pI2CHandle->pI2Cx->OAR1 |= (temp_register << OAR1_BIT_ADD);

	temp_register = 0;
	uint16_t ccr_value = 0;

	pI2CHandle->pI2Cx->CCR = 0;
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
		ccr_value = pclk1 / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
	} else {

		temp_register = SET;
		pI2CHandle->pI2Cx->CCR |= (temp_register << CCR_BIT_FS);

		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2) {
			ccr_value = pclk1 / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		} else {
		    pI2CHandle->pI2Cx->CCR |= (SET << CCR_BIT_DUTY);
			ccr_value = pclk1 / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}


	}

	pI2CHandle->pI2Cx->CCR &= ~(0xFFF << CCR_BIT_CCR);
	pI2CHandle->pI2Cx->CCR |= (ccr_value << CCR_BIT_CCR);

	temp_register = 0;

	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
		temp_register = freq + 1;
	} else {
		temp_register = ((freq * 3) / 10) + 1;
	}

	pI2CHandle->pI2Cx->TRISE = (temp_register & 0x3F);

}

void I2C_PeriClockControl(I2C_Handle_t *pI2CHandle, uint8_t isEnable){
	if(isEnable) {
		if(pI2CHandle->pI2Cx == I2C1) {
			I2C1_PCLK_EN();
		} else if(pI2CHandle->pI2Cx == I2C2) {
			I2C2_PCLK_EN();
		} else if(pI2CHandle->pI2Cx == I2C3) {
			I2C3_PCLK_EN();
		}
	} else {
		if(pI2CHandle->pI2Cx == I2C1) {
			I2C1_PCLK_DI();
		} else if(pI2CHandle->pI2Cx == I2C2) {
			I2C2_PCLK_DI();
		} else if(pI2CHandle->pI2Cx == I2C3) {
			I2C3_PCLK_DI();
		}
	}
}


void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t isEnable){
	if (isEnable) {
		pI2Cx->CR1 |= (1 << CR1_BIT_PE);
	} else {
		pI2Cx->CR1 &= ~(1 << CR1_BIT_PE);
	}
}


void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi){
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

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	uint8_t iprx = (uint8_t) (IRQNumber  / 4);
	uint8_t prix = IRQNumber % 4;
	uint8_t shift = (8 * prix) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_IPR_BASEADDR + (iprx)) |= (IRQPriority << shift);
}

static void I2C_generateStartCondition(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR1 |= (SET << CR1_BIT_START);

}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t len, uint8_t slaveAddr, uint8_t sr){

	I2C_generateStartCondition(pI2CHandle->pI2Cx);

	while(!(pI2CHandle->pI2Cx->SR1 & (1 << SR1_BIT_SB)));

	slaveAddr = slaveAddr << 1;
	slaveAddr &= ~(1);

	pI2CHandle->pI2Cx->DR = slaveAddr;

	while(!(pI2CHandle->pI2Cx->SR1 & (1 << SR1_BIT_ADDR)));


	// Clear the ADDR flag
	uint16_t read1 = pI2CHandle->pI2Cx->SR1;
	uint16_t read2 = pI2CHandle->pI2Cx->SR2;

	(void) read1;
	(void) read2;

	while (len > 0) {

		while(!(pI2CHandle->pI2Cx->SR1 & (1 << SR1_BIT_TXE)));

		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		len--;
		pTxbuffer++;
	}

	while(!(pI2CHandle->pI2Cx->SR1 & (1 << SR1_BIT_TXE)) || !(pI2CHandle->pI2Cx->SR1 & (1 << SR1_BIT_BTF)));

	pI2CHandle->pI2Cx->CR1 |= (1 << CR1_BIT_STOP);


}

