/*
 * stm32f407x_i2_driver.h
 *
 *  Created on: Jan 24, 2026
 *      Author: ardademirkran
 */

#ifndef INC_STM32F407X_I2C_DRIVER_H_
#define INC_STM32F407X_I2C_DRIVER_H_

#include "stm32f407xx.h"


/*
 * I2C CR1 Bit Position Macros
 */
#define CR1_BIT_PE				0
#define CR1_BIT_SMBUS			1
#define	CR1_BIT_SMBTYPE			3
#define CR1_BIT_ENARP			4
#define CR1_BIT_ENPEC			5
#define CR1_BIT_ENGC			6
#define CR1_BIT_NOSTRETCH		7
#define CR1_BIT_START			8
#define CR1_BIT_STOP			9
#define CR1_BIT_ACK				10
#define CR1_BIT_POS				11
#define CR1_BIT_PEC				12
#define CR1_BIT_ALERT			13
#define CR1_BIT_SWRST			15


/*
 * I2C CR2 Bit Position Macros
 */
#define CR2_BIT_FREQ 			5
#define CR2_BIT_ITERREN			8
#define CR2_BIT_ITEVTEN			9
#define CR2_BIT_ITBUFEN			10
#define CR2_BIT_DMAEN			11
#define CR2_BIT_LAST			12


/*
 * I2C_OAR1 Bit Position Macros
 */
#define OAR1_BIT_ADD0			0
#define OAR1_BIT_ADD			7
#define OAR1_BIT_ADD_10_BIT		9
#define OAR1_BIT_ADDMODE		15

/*
 * I2C_OAR2 Bit Position Macros
 */
#define OAR2_BIT_ENDUAL			0
#define OAR2_BIT_ADD			7

/*
 * I2C_DR Bit Position Macros
 */
#define DR_BIT_DR				7


/*
 * I2C_SR1 Bit Position Macros
 */
#define SR1_BIT_SB				0
#define SR1_BIT_ADDR			1
#define SR1_BIT_BTF				2
#define SR1_BIT_ADD10			3
#define SR1_BIT_STOPF			4
#define SR1_BIT_RXNE			6
#define SR1_BIT_TXE				7
#define SR1_BIT_BERR			8
#define SR1_BIT_ARLO			9
#define SR1_BIT_AF				10
#define SR1_BIT_OVR				11
#define SR1_BIT_PECERR			12
#define SR1_BIT_TIMEOUT			14
#define SR1_BIT_SMBALERT		15

/*
 * I2C_CCR Bit Position Macros
 */
#define	CCR_BIT_CCR				11
#define CCR_BIT_DUTY			14
#define CCR_BIT_FS				15


/*
 * I2C_TRISE Bit Position Macros
 */
#define TRISE_BIT_TRISE			5

/*
 * I2C_FLTR Bit Position Macros
 */

#define FLTR_BIT_DNF			3
#define FLTR_BIT_ANOF			4





typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t  I2C_DeviceAddress;
	uint8_t  I2C_AckControl;
	uint8_t  I2C_FMDutyCycle;

} I2C_Config_t;

/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM		100000
#define I2C_SCL_SPEED_FM		400000

/*
 * @I2C_AckControl
 */
#define I2C_ACK_ENABLE			1
#define I2C_ACK_DISABLE			0

/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2			0
#define I2C_FM_DUTY_16_9		1

typedef struct {
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
} I2C_Handle_t;


void I2C_PeriClockControl(I2C_Handle_t *pI2CHandle, uint8_t isEnable);


void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t len, uint8_t slaveAddr, uint8_t sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr);
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr);

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv);


#endif /* INC_STM32F407X_I2C_DRIVER_H_ */
