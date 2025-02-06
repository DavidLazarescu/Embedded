/*
 * PCF85176_Driver.h
 *
 *  Created on: Feb 6, 2025
 *      Author: David
 */

#include "stm32c0xx_hal.h"

#ifndef INC_PCF85176_DRIVER_H_
#define INC_PCF85176_DRIVER_H_

// Defines
#define PCF85176_ADDRESS 0b01110000

typedef struct {
	I2C_HandleTypeDef* i2cHandle;
} PCF85176;

HAL_StatusTypeDef PCF85176_Initalise(PCF85176* dev, I2C_HandleTypeDef* i2cHandle);
HAL_StatusTypeDef PCF85176_DisplayString(PCF85176* dev, char* str, uint8_t decimalPointPos);

#endif /* INC_PCF85176_DRIVER_H_ */
