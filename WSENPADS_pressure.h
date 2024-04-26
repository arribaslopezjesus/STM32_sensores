/*
 * WSENPADS_pressure.h
 *
 *  Created on: 10 ago. 2021
 *      Author: JAL
 */

#include "stm32f4xx_hal.h"
#include "../conf.h"

#ifndef SRC_WSENPADS_PRESSURE_H_
#define SRC_WSENPADS_PRESSURE_H_

#define WSEN_PADS_INT_I2C_ADDRESS (0x5C<<1)
#define WSEN_PADS_EXT_I2C_ADDRESS (0x5D<<1)

#endif /* SRC_WSENPADS_PRESSURE_H_ */


uint8_t INIT_WSENPADS(I2C_HandleTypeDef *lhi2c1, uint8_t I2C_Adress);
void PRESTEMP_WSENPADS(I2C_HandleTypeDef *lhi2c1, param_sens *DatosStruct, uint8_t I2C_Adress);
void DATA_PADS_DISP(I2C_HandleTypeDef *lhi2c1, uint8_t I2C_Adress);
void OFF_WSENPADS(I2C_HandleTypeDef *lhi2c1, uint8_t I2C_Adress);

