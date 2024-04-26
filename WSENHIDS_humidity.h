/*
 * WSENHIDS_humidity.h
 *
 *  Created on: 10 ago. 2021
 *      Author: JAL
 */

#include "stm32f4xx_hal.h"
#include "../conf.h"

#ifndef SRC_UTILITIES_WSENHIDS_HUMIDITY_H_
#define SRC_UTILITIES_WSENHIDS_HUMIDITY_H_

#define WSEN_HIDS_I2C_ADDRESS (0x5F<<1)

#endif /* SRC_UTILITIES_WSENHIDS_HUMIDITY_H_ */


uint8_t INIT_WSENHIDS(I2C_HandleTypeDef *lhi2c1);
void HUMTEMP_WSENHIDS(I2C_HandleTypeDef *lhi2c1, param_sens *DatosStruct);
void DATA_HIDS_DISP(I2C_HandleTypeDef *lhi2c1);
void OFF_WSENHIDS(I2C_HandleTypeDef *lhi2c1);

