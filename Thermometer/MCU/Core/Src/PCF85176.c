/*
 * PCF85176.c
 *
 *  Created on: Feb 6, 2025
 *      Author: David
 */

#include <string.h>

#include "PCF85176.h"

uint8_t getCodeForNum(uint8_t num);

HAL_StatusTypeDef PCF85176_Initalise(PCF85176* dev, I2C_HandleTypeDef* i2cHandle)
{
	dev->i2cHandle = i2cHandle;

	uint8_t controlBytes[5] = { 0b11001001, 0b10000000, 0b11100000, 0b11111000, 0b01110000 };
	return HAL_I2C_Master_Transmit(dev->i2cHandle, PCF85176_ADDRESS, controlBytes, sizeof(controlBytes), HAL_MAX_DELAY);
}

HAL_StatusTypeDef PCF85176_DisplayString(PCF85176* dev, char* str, uint8_t decimalPointPos)
{
	// Only strings of characters and a decimal point position of 1 or 2 are accepted
	int len = strlen(str);
	if(len > 4 || (decimalPointPos > 2 || decimalPointPos < 1))
		return HAL_ERROR;

	uint8_t segmentCodes[3];
	for(int i = 0; i < strlen(str); ++i)
	{
		char c = str[i];
		segmentCodes[i] = getCodeForNum(c);
	}

	uint8_t data[5] = {0};
	data[0] = 0x00; // Skip Control byte

	// Pack the 7bit codes into the array without any gaps
	uint8_t bitIndex = 0;
	for(int i = 0; i < sizeof(segmentCodes); ++i)
	{
		uint8_t code = segmentCodes[i];
		uint8_t bytePos = bitIndex / 8;
		uint8_t bitPos = bitIndex % 8;

		data[bytePos + 1] |= (code >> bitPos);
		if(bitPos > 1)
			data[bytePos + 2] |= (code << (8 - bitPos));

		bitIndex += 7;
	}

	if(decimalPointPos == 1)
		data[3 + 1] = 0b00001000;
	else if(decimalPointPos == 2)
		data[3 + 1] = 0b00000100;

	return HAL_I2C_Master_Transmit(dev->i2cHandle, PCF85176_ADDRESS, data, sizeof(data), HAL_MAX_DELAY);
}

uint8_t getCodeForNum(uint8_t num)
{
	switch((char)num) {
		case '0':
			return 0b11111100;
		case '1':
			return 0b01100000;
		case '2':
			return 0b11011010;
		case '3':
			return 0b11110010;
		case '4':
			return 0b01100110;
		case '5':
			return 0b10110110;
		case '6':
			return 0b10111110;
		case '7':
			return 0b11100000;
		case '8':
			return 0b11111110;
		case '9':
			return 0b11110110;
		default:
			return 0;
	}

	return 0;
}


