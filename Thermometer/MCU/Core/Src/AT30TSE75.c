/*
 * AT30TSE75.c
 *
 *  Created on: Feb 6, 2025
 *      Author: David
 */


#include "AT30TSE75.h"
#include <string.h>

HAL_StatusTypeDef AT30TSE75_Initalise(AT30TSE75* dev, I2C_HandleTypeDef* i2cHandle)
{
	dev->i2cHandle = i2cHandle;

	uint8_t conf_data[2];
	if(HAL_I2C_Mem_Read(dev->i2cHandle, AT30TSE75_ADDRESS, 0x01, I2C_MEMADD_SIZE_8BIT, conf_data, 2, HAL_MAX_DELAY) != HAL_OK)
		return HAL_ERROR;

	conf_data[0] |= 0x20;
	return HAL_I2C_Mem_Write(dev->i2cHandle, AT30TSE75_ADDRESS, 0x01, I2C_MEMADD_SIZE_8BIT, conf_data, 2, HAL_MAX_DELAY);
}

float AT30TSE75_ReadTemperature(AT30TSE75* dev)
{
	uint8_t currData[2] = {0};
	if(HAL_I2C_Mem_Read(dev->i2cHandle, AT30TSE75_ADDRESS, 0x01, I2C_MEMADD_SIZE_8BIT, currData, 2, HAL_MAX_DELAY) != HAL_OK) {
		return -1;
	}

	// Set the pointer to the temperature register for the following reading
	uint8_t temp_reg = 0x00;
	HAL_I2C_Master_Transmit(dev->i2cHandle, AT30TSE75_ADDRESS, &temp_reg, 1, HAL_MAX_DELAY);

	uint8_t temp_data[2] = {0};
	HAL_StatusTypeDef res = HAL_I2C_Master_Receive(dev->i2cHandle, AT30TSE75_ADDRESS, temp_data, 2, HAL_MAX_DELAY);
	if(res != HAL_OK){
		return -1;
	}

	uint16_t temp_raw = (temp_data[0] << 8) | temp_data[1];
	float temperature = temp_raw / 256.0;

	return temperature;
}
