/*
 * AT30TSE75.h
 *
 *  Created on: Feb 6, 2025
 *      Author: David
 */

#include "stm32c0xx_hal.h"

#ifndef INC_AT30TSE75_H_
#define INC_AT30TSE75_H_

// Defines
#define AT30TSE75_ADDRESS 0x48 << 1

typedef struct {
	I2C_HandleTypeDef* i2cHandle;
} AT30TSE75;

HAL_StatusTypeDef AT30TSE75_Initalise(AT30TSE75* dev, I2C_HandleTypeDef* i2cHandle);
float AT30TSE75_ReadTemperature(AT30TSE75* dev);

#endif /* INC_AT30TSE75_H_ */
