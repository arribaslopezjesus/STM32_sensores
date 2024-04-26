/*
 * AS7341.h
 *
 *  Created on: 26 ago. 2022
 *      Author: jesuc
 */

#ifndef SRC_UTILITIES_AS7341_H_
#define SRC_UTILITIES_AS7341_H_

#include "stm32f4xx_hal.h"
#include "../conf.h"

#endif /* SRC_UTILITIES_AS7341_H_ */

uint8_t AS7341_Init(I2C_HandleTypeDef *lhi2c1);
void AS7341_Read_Channels(I2C_HandleTypeDef *lhi2c1, param_AS7341 *datosStruct);
void AS7341_Off(I2C_HandleTypeDef *lhi2c1);

