/*
 * AS7341.c
 *
 *  Created on: 26 ago. 2022
 *      Author: JAL
 */

#include "AS7341.h"
#include "string.h"
#include "printf.h"
#include "../conf.h"
#include "math.h"
#include "Sim7600.h"

char msj4[200];

/**
 * @brief Función que detecta e inicializa el sensor AS7341.
 * @param lhi2c1 Estructura que maneja los parametros necesarios para configurar y utilizar el i2c1
 * @return booleano 1:Inicialización correcta 0:Inicialización incorrecta
 */
uint8_t AS7341_Init(I2C_HandleTypeDef *lhi2c1)
{
	uint8_t ENABLE = 0;
	uint8_t ID_AS7341 = 0;
	uint8_t ATIME = 29;
	uint8_t ASTEP_L = 87;
	uint8_t ASTEP_H = 2;
	uint8_t ASTEP[2] = {0};
	uint16_t ASTEP_16 = 0;
	uint8_t CFG8 = 0;

	//Power On
	if(HAL_I2C_Mem_Read(lhi2c1, 0x39<<1, 0x80, I2C_MEMADD_SIZE_8BIT, &ENABLE, 1, 10000) != HAL_OK)
	{
		Error_Handler();
		return 0;
	}
	e_sprintf(msj4, "ENABLE: %d\n", ENABLE);
	printf(msj4);

	ENABLE |= 0x01;
	if(HAL_I2C_Mem_Write(lhi2c1, 0x39<<1, 0x80, I2C_MEMADD_SIZE_8BIT, &ENABLE, 1, 10000) != HAL_OK)
	{
		Error_Handler();
	}

	if(HAL_I2C_Mem_Read(lhi2c1, 0x39<<1, 0x80, I2C_MEMADD_SIZE_8BIT, &ENABLE, 1, 10000) != HAL_OK)
	{
		Error_Handler();
	}
	e_sprintf(msj4, "ENABLE: %d\n", ENABLE);
	printf(msj4);
	printf("Enable AS7341\n");

	//Lectura del ID
	if(HAL_I2C_Mem_Read(lhi2c1, 0x39<<1, 0x92, I2C_MEMADD_SIZE_8BIT, &ID_AS7341, 1, 10000) != HAL_OK)
	{
		Error_Handler();
	}
	e_sprintf(msj4, "ID: %d\n", ID_AS7341);
	printf(msj4);

	//Configuración de ATIME
	if(HAL_I2C_Mem_Write(lhi2c1, 0x39<<1, 0x81, I2C_MEMADD_SIZE_8BIT, &ATIME, 1, 10000) != HAL_OK)
	{
		Error_Handler();
	}
	e_sprintf(msj4, "ATIME: %d\n", ATIME);
	printf(msj4);

	//Configuración de ASTEP
	if(HAL_I2C_Mem_Write(lhi2c1, 0x39<<1, 0xCA, I2C_MEMADD_SIZE_8BIT, &ASTEP_L, 1, 10000) != HAL_OK)
	{
		Error_Handler();
	}

	if(HAL_I2C_Mem_Write(lhi2c1, 0x39<<1, 0xCB, I2C_MEMADD_SIZE_8BIT, &ASTEP_H, 1, 10000) != HAL_OK)
	{
		Error_Handler();
	}

	if(HAL_I2C_Mem_Read(lhi2c1, 0x39<<1, 0xCA, I2C_MEMADD_SIZE_8BIT, ASTEP, 2, 10000) != HAL_OK)
	{
		Error_Handler();
	}
	ASTEP_16 = (ASTEP[1] << 8) + ASTEP[0];
	e_sprintf(msj4, "ASTEP: %d\n", ASTEP_16);
	printf(msj4);
	printf("Config ATIME and ASTEP\n");

	//Configuración de la ganancia
	if(HAL_I2C_Mem_Read(lhi2c1, 0x39<<1, 0xB1, I2C_MEMADD_SIZE_8BIT, &CFG8, 1, 10000) != HAL_OK)
	{
		Error_Handler();
	}

	CFG8 |= 0x04;
	if(HAL_I2C_Mem_Write(lhi2c1, 0x39<<1, 0xB1, I2C_MEMADD_SIZE_8BIT, &CFG8, 1, 10000) != HAL_OK)
	{
		Error_Handler();
	}
	if(HAL_I2C_Mem_Read(lhi2c1, 0x39<<1, 0xB1, I2C_MEMADD_SIZE_8BIT, &CFG8, 1, 10000) != HAL_OK)
	{
		Error_Handler();
	}

	return 1;
}

/**
 * @brief Función que lee y calcula los datos de todos los canales del sensor AS7341.
 * @param lhi2c1 Estructura que maneja los parametros necesarios para configurar y utilizar el i2c1
 *        datosStruct Estructura que contiene los datos de los diferentes canales del sensor AS7341
 * @return void
 */
void AS7341_Read_Channels(I2C_HandleTypeDef *lhi2c1, param_AS7341 *datosStruct)
{

	uint8_t ENABLE = 0;
	uint8_t WRITE_SMUX = 0;
	uint8_t ADC0_ADC5[12] = {0};
	uint8_t STATUS_2 = 0;
	uint8_t STATUS_6 = 0;
	uint8_t STATUS = 0;
	uint8_t STAT = 0;
	uint8_t F1_F4_CLEAR_NIR[20] = {0x30, 0x01, 0x00, 0x00, 0x00, 0x42, 0x00, 0x00, 0x50, 0x00, 0x00, 0x00, 0x20, 0x04, 0x00, 0x30, 0x01, 0x50, 0x00, 0x06};
	uint8_t F5_F8_CLEAR_NIR[20] = {0x00, 0x00, 0x00, 0x40, 0x02, 0x00, 0x10, 0x03, 0x50, 0x10, 0x03, 0x00, 0x00, 0x00, 0x24, 0x00, 0x00, 0x50, 0x00, 0x06};
	uint8_t READ_F1_F4[20] = {0};
	uint8_t GAIN_STATUS = 0;
	float GAIN[11] = {0.5, 1, 2, 4, 8, 16, 32, 64, 128, 256, 512};
	uint16_t TEMP_INT = 0;
	uint16_t ADC_0, ADC_1, ADC_2, ADC_3, ADC_4, ADC_5 = 0;
	float F1, F2, F3, F4, F5, F6, F7, F8, CLEAR, NIR = 0;
	char F1_s [10] = {0};
	char F2_s [10] = {0};
	char F3_s [10] = {0};
	char F4_s [10] = {0};
	char F5_s [10] = {0};
	char F6_s [10] = {0};
	char F7_s [10] = {0};
	char F8_s [10] = {0};
	char CLEAR_s [10] = {0};
	char NIR_s [10] = {0};
	uint8_t i = 0;

	//Deshabilitar medida espectral
	if(HAL_I2C_Mem_Read(lhi2c1, 0x39<<1, 0x80, I2C_MEMADD_SIZE_8BIT, &ENABLE, 1, 10000) != HAL_OK)
	{
		Error_Handler();
	}
	e_sprintf(msj4, "ENABLE: %d\n", ENABLE);
	printf(msj4);

	ENABLE &= 0xFD;
	if(HAL_I2C_Mem_Write(lhi2c1, 0x39<<1, 0x80, I2C_MEMADD_SIZE_8BIT, &ENABLE, 1, 10000) != HAL_OK)
	{
		Error_Handler();
	}

	if(HAL_I2C_Mem_Read(lhi2c1, 0x39<<1, 0x80, I2C_MEMADD_SIZE_8BIT, &ENABLE, 1, 10000) != HAL_OK)
	{
		Error_Handler();
	}
	e_sprintf(msj4, "ENABLE: %d\n", ENABLE);
	printf(msj4);
	printf("Disable Spectral Measurement\n");

	//Configurar el comando WRITE en el SMUX
	WRITE_SMUX = 0x10;
	if(HAL_I2C_Mem_Write(lhi2c1, 0x39<<1, 0xAF, I2C_MEMADD_SIZE_8BIT, &WRITE_SMUX, 1, 10000) != HAL_OK)
	{
		Error_Handler();
	}
	if(HAL_I2C_Mem_Read(lhi2c1, 0x39<<1, 0xAF, I2C_MEMADD_SIZE_8BIT, &WRITE_SMUX, 1, 10000) != HAL_OK)
	{
		Error_Handler();
	}
	e_sprintf(msj4, "WRITE_SMUX: %d\n", WRITE_SMUX);
	printf(msj4);
	printf("SMUX_WRITE\n");

	//Configurar los multiplexadores para la lectura de F1 F2 F3 F4 CLEAR y NIR
	if(HAL_I2C_Mem_Write(lhi2c1, 0x39<<1, 0x00, I2C_MEMADD_SIZE_8BIT, F1_F4_CLEAR_NIR, 20, 10000) != HAL_OK)
	{
		Error_Handler();
	}

	if(HAL_I2C_Mem_Read(lhi2c1, 0x39<<1, 0x00, I2C_MEMADD_SIZE_8BIT, READ_F1_F4, 20, 10000) != HAL_OK)
	{
		Error_Handler();
	}
	printf("SET F1_F4_CLEAR_NIR to ADC\n");

	//Habilitar el multiplexor
	if(HAL_I2C_Mem_Read(lhi2c1, 0x39<<1, 0x80, I2C_MEMADD_SIZE_8BIT, &ENABLE, 1, 10000) != HAL_OK)
	{
		Error_Handler();
	}
	e_sprintf(msj4, "ENABLE: %d\n", ENABLE);
	printf(msj4);

	ENABLE |= 0x10;
	if(HAL_I2C_Mem_Write(lhi2c1, 0x39<<1, 0x80, I2C_MEMADD_SIZE_8BIT, &ENABLE, 1, 10000) != HAL_OK)
	{
		Error_Handler();
	}

	if(HAL_I2C_Mem_Read(lhi2c1, 0x39<<1, 0x80, I2C_MEMADD_SIZE_8BIT, &ENABLE, 1, 10000) != HAL_OK)
	{
		Error_Handler();
	}
	e_sprintf(msj4, "ENABLE: %d\n", ENABLE);
	printf(msj4);
	printf("SMUX Enable\n");

	//Habilitar medida espectral
	ENABLE |= 0x02;
	if(HAL_I2C_Mem_Write(lhi2c1, 0x39<<1, 0x80, I2C_MEMADD_SIZE_8BIT, &ENABLE, 1, 10000) != HAL_OK)
	{
		Error_Handler();
	}
	if(HAL_I2C_Mem_Read(lhi2c1, 0x39<<1, 0x80, I2C_MEMADD_SIZE_8BIT, &ENABLE, 1, 10000) != HAL_OK)
	{
		Error_Handler();
	}
	e_sprintf(msj4, "ENABLE: %d\n", ENABLE);
	printf(msj4);
	printf("Enable Spectral Measurement\n");

	//Lecturas de estado
	if(HAL_I2C_Mem_Read(lhi2c1, 0x39<<1, 0x93, I2C_MEMADD_SIZE_8BIT, &STATUS, 1, 10000) != HAL_OK)
	{
		Error_Handler();
	}
	e_sprintf(msj4, "STATUS: %d\n", STATUS);
	printf(msj4);

	if(HAL_I2C_Mem_Read(lhi2c1, 0x39<<1, 0xA7, I2C_MEMADD_SIZE_8BIT, &STATUS_6, 1, 10000) != HAL_OK)
	{
		Error_Handler();
	}
	e_sprintf(msj4, "STATUS_6: %d\n", STATUS_6);
	printf(msj4);

	//Comprobacion de datos disponible
	do
	{
		if(HAL_I2C_Mem_Read(lhi2c1, 0x39<<1, 0xA3, I2C_MEMADD_SIZE_8BIT, &STATUS_2, 1, 10000) != HAL_OK)
		{
			Error_Handler();
		}
		HAL_Delay(100);
		i++;
		if(i == 200)
			return;
	}while((STATUS_2 & 0x40) != 0x40);

	//Lectura del valor de gananacia con el que se ha tomado la muestra
	if(HAL_I2C_Mem_Read(lhi2c1, 0x39<<1, 0x94, I2C_MEMADD_SIZE_8BIT, &datosStruct->ASTATUS_1, 1, 10000) != HAL_OK)
	{
		Error_Handler();
	}
	e_sprintf(msj4, "ASTATUS_1: %d\n", datosStruct->ASTATUS_1);
	printf(msj4);

	//Lectura de los datos de luminosidad brutos
	if(HAL_I2C_Mem_Read(lhi2c1, 0x39<<1, 0x95, I2C_MEMADD_SIZE_8BIT, ADC0_ADC5, 12, 10000) != HAL_OK)
	{
		Error_Handler();
	}

	//Calculo de los datos de luminosidad
	ADC_0 = (ADC0_ADC5[1] << 8) | (ADC0_ADC5[0]);
	ADC_1 = (ADC0_ADC5[3] << 8) | (ADC0_ADC5[2]);
	ADC_2 = (ADC0_ADC5[5] << 8) | (ADC0_ADC5[4]);
	ADC_3 = (ADC0_ADC5[7] << 8) | (ADC0_ADC5[6]);
	ADC_4 = (ADC0_ADC5[9] << 8) | (ADC0_ADC5[8]);
	ADC_5 = (ADC0_ADC5[11] << 8) | (ADC0_ADC5[10]);

	GAIN_STATUS = datosStruct->ASTATUS_1 & 0x0F;
	TEMP_INT = 50 * GAIN[GAIN_STATUS];

	F1 = (float)(((uint16_t)(ADC_0)) / (float)((uint16_t)(TEMP_INT)));
	F2 = (float)(((uint16_t)(ADC_1)) / (float)((uint16_t)(TEMP_INT)));
	F3 = (float)(((uint16_t)(ADC_2)) / (float)((uint16_t)(TEMP_INT)));
	F4 = (float)(((uint16_t)(ADC_3)) / (float)((uint16_t)(TEMP_INT)));
	CLEAR = (float)(((uint16_t)(ADC_4)) / (float)((uint16_t)(TEMP_INT)));
	NIR = (float)(((uint16_t)(ADC_5)) / (float)((uint16_t)(TEMP_INT)));

	datosStruct->F1_AS7341 = F1;
	datosStruct->F2_AS7341 = F2;
	datosStruct->F3_AS7341 = F3;
	datosStruct->F4_AS7341 = F4;

	myFtoa(F1, F1_s);
	myFtoa(F2, F2_s);
	myFtoa(F3, F3_s);
	myFtoa(F4, F4_s);
	myFtoa(CLEAR, CLEAR_s);
	myFtoa(NIR, NIR_s);

	e_sprintf(msj4, "[AS7341.c] Medidas de los diferentes canales del AS7341. Formato RAW y BASIC\n");
	debug_serie(msj4);
	e_sprintf(msj4, "\nF1_ADC = %d F1 = %s\nF2_ADC = %d F2 = %s\nF3_ADC = %d F3 = %s\nF4_ADC = %d F4 = %s\n", ADC_0, F1_s, ADC_1, F2_s, ADC_2, F3_s, ADC_3, F4_s);
	debug_serie(msj4);

	//Deshabilitar la medición espectrl
	if(HAL_I2C_Mem_Read(lhi2c1, 0x39<<1, 0x80, I2C_MEMADD_SIZE_8BIT, &ENABLE, 1, 10000) != HAL_OK)
	{
		Error_Handler();
	}
	e_sprintf(msj4, "ENABLE: %d\n", ENABLE);
	printf(msj4);

	ENABLE &= 0xFD;
	if(HAL_I2C_Mem_Write(lhi2c1, 0x39<<1, 0x80, I2C_MEMADD_SIZE_8BIT, &ENABLE, 1, 10000) != HAL_OK)
	{
		Error_Handler();
	}

	if(HAL_I2C_Mem_Read(lhi2c1, 0x39<<1, 0x80, I2C_MEMADD_SIZE_8BIT, &ENABLE, 1, 10000) != HAL_OK)
	{
		Error_Handler();
	}
	e_sprintf(msj4, "ENABLE: %d\n", ENABLE);
	printf(msj4);
	printf("Disable Spectral Measurement\n");

	if(HAL_I2C_Mem_Read(lhi2c1, 0x39<<1, 0xAF, I2C_MEMADD_SIZE_8BIT, &WRITE_SMUX, 1, 10000) != HAL_OK)
	{
		Error_Handler();
	}
	WRITE_SMUX |= 0x10;
	if(HAL_I2C_Mem_Write(lhi2c1, 0x39<<1, 0xAF, I2C_MEMADD_SIZE_8BIT, &WRITE_SMUX, 1, 10000) != HAL_OK)
	{
		Error_Handler();
	}
	if(HAL_I2C_Mem_Read(lhi2c1, 0x39<<1, 0xAF, I2C_MEMADD_SIZE_8BIT, &WRITE_SMUX, 1, 10000) != HAL_OK)
	{
		Error_Handler();
	}
	e_sprintf(msj4, "WRITE_SMUX: %d\n", WRITE_SMUX);
	printf(msj4);
	printf("SMUX_WRITE\n");

	if(HAL_I2C_Mem_Write(lhi2c1, 0x39<<1, 0x00, I2C_MEMADD_SIZE_8BIT, F5_F8_CLEAR_NIR, 20, 10000) != HAL_OK)
	{
		Error_Handler();
	}

	if(HAL_I2C_Mem_Read(lhi2c1, 0x39<<1, 0x00, I2C_MEMADD_SIZE_8BIT, READ_F1_F4, 20, 10000) != HAL_OK)
	{
		Error_Handler();
	}
	printf("SET F5_F8_CLEAR_NIR to ADC\n");

	if(HAL_I2C_Mem_Read(lhi2c1, 0x39<<1, 0x80, I2C_MEMADD_SIZE_8BIT, &ENABLE, 1, 10000) != HAL_OK)
	{
		Error_Handler();
	}
	e_sprintf(msj4, "ENABLE: %d\n", ENABLE);
	printf(msj4);

	ENABLE |= 0x10;
	if(HAL_I2C_Mem_Write(lhi2c1, 0x39<<1, 0x80, I2C_MEMADD_SIZE_8BIT, &ENABLE, 1, 10000) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_Delay(1000);
	if(HAL_I2C_Mem_Read(lhi2c1, 0x39<<1, 0x80, I2C_MEMADD_SIZE_8BIT, &ENABLE, 1, 10000) != HAL_OK)
	{
		Error_Handler();
	}
	e_sprintf(msj4, "ENABLE: %d\n", ENABLE);
	printf(msj4);
	printf("SMUX Enable\n");

	ENABLE |= 0x02;
	if(HAL_I2C_Mem_Write(lhi2c1, 0x39<<1, 0x80, I2C_MEMADD_SIZE_8BIT, &ENABLE, 1, 10000) != HAL_OK)
	{
		Error_Handler();
	}
	if(HAL_I2C_Mem_Read(lhi2c1, 0x39<<1, 0x80, I2C_MEMADD_SIZE_8BIT, &ENABLE, 1, 10000) != HAL_OK)
	{
		Error_Handler();
	}
	e_sprintf(msj4, "ENABLE: %d\n", ENABLE);
	printf(msj4);
	printf("Enable Spectral Measurement\n");


	if(HAL_I2C_Mem_Read(lhi2c1, 0x39<<1, 0x71, I2C_MEMADD_SIZE_8BIT, &STAT, 1, 10000) != HAL_OK)
	{
		Error_Handler();
	}
	e_sprintf(msj4, "STAT: %d\n", STAT);
	printf(msj4);

	if(HAL_I2C_Mem_Read(lhi2c1, 0x39<<1, 0x93, I2C_MEMADD_SIZE_8BIT, &STATUS, 1, 10000) != HAL_OK)
	{
		Error_Handler();
	}
	e_sprintf(msj4, "STATUS: %d\n", STATUS);
	printf(msj4);

	if(HAL_I2C_Mem_Read(lhi2c1, 0x39<<1, 0xA7, I2C_MEMADD_SIZE_8BIT, &STATUS_6, 1, 10000) != HAL_OK)
	{
		Error_Handler();
	}
	e_sprintf(msj4, "STATUS_6: %d\n", STATUS_6);
	printf(msj4);
	
	i = 0;
	do
	{
		if(HAL_I2C_Mem_Read(lhi2c1, 0x39<<1, 0xA3, I2C_MEMADD_SIZE_8BIT, &STATUS_2, 1, 10000) != HAL_OK)
		{
			Error_Handler();
		}
		HAL_Delay(100);
		i++;
		if(i == 200)
			return;

	}while((STATUS_2 & 0x40) != 0x40);

	if(HAL_I2C_Mem_Read(lhi2c1, 0x39<<1, 0x94, I2C_MEMADD_SIZE_8BIT, &datosStruct->ASTATUS_2, 1, 10000) != HAL_OK)
	{
		Error_Handler();
	}
	e_sprintf(msj4, "ASTATUS_2: %d\n", datosStruct->ASTATUS_2);
	printf(msj4);

	i = 0;


	if(HAL_I2C_Mem_Read(lhi2c1, 0x39<<1, 0x95, I2C_MEMADD_SIZE_8BIT, ADC0_ADC5, 12, 10000) != HAL_OK)
	{
		Error_Handler();
	}

	ADC_0 = (ADC0_ADC5[1] << 8) | (ADC0_ADC5[0]);
	ADC_1 = (ADC0_ADC5[3] << 8) | (ADC0_ADC5[2]);
	ADC_2 = (ADC0_ADC5[5] << 8) | (ADC0_ADC5[4]);
	ADC_3 = (ADC0_ADC5[7] << 8) | (ADC0_ADC5[6]);
	ADC_4 = (ADC0_ADC5[9] << 8) | (ADC0_ADC5[8]);
	ADC_5 = (ADC0_ADC5[11] << 8) | (ADC0_ADC5[10]);

	GAIN_STATUS = datosStruct->ASTATUS_2 & 0x0F;
	TEMP_INT = 50 * GAIN[GAIN_STATUS];

	F5 = (float)(((uint16_t)(ADC_0)) / (float)((uint16_t)(TEMP_INT)));
	F6 = (float)(((uint16_t)(ADC_1)) / (float)((uint16_t)(TEMP_INT)));
	F7 = (float)(((uint16_t)(ADC_2)) / (float)((uint16_t)(TEMP_INT)));
	F8 = (float)(((uint16_t)(ADC_3)) / (float)((uint16_t)(TEMP_INT)));
	CLEAR = (float)(((uint16_t)(ADC_4)) / (float)((uint16_t)(TEMP_INT)));
	NIR = (float)(((uint16_t)(ADC_5)) / (float)((uint16_t)(TEMP_INT)));

	datosStruct->F5_AS7341 = F5;
	datosStruct->F6_AS7341 = F6;
	datosStruct->F7_AS7341 = F7;
	datosStruct->F8_AS7341 = F8;
	datosStruct->CLEAR_AS7341 = CLEAR;
	datosStruct->NIR_AS7341 = NIR;

	myFtoa(F5, F5_s);
	myFtoa(F6, F6_s);
	myFtoa(F7, F7_s);
	myFtoa(F8, F8_s);
	myFtoa(CLEAR, CLEAR_s);
	myFtoa(NIR, NIR_s);

	e_sprintf(msj4, "F5_ADC = %d F5 = %s\nF6_ADC = %d F6 = %s\nF7_ADC = %d F7 = %s\nF8_ADC = %d F8 = %s\nCLEAR_ADC = %d CLEAR = %s\nNIR_ADC = %d NIR = %s\n\n", ADC_0, F5_s, ADC_1, F6_s, ADC_2, F7_s, ADC_3, F8_s, ADC_4, CLEAR_s, ADC_5, NIR_s);
	debug_serie(msj4);
}

/**
 * @brief Función que desactiva el sensor AS7341
 * @param lhi2c1 Estructura que maneja los parametros necesarios para configurar y utilizar el i2c1
 * @return void
 */
void AS7341_Off(I2C_HandleTypeDef *lhi2c1)
{
	uint8_t ENABLE = 0;
	if(HAL_I2C_Mem_Read(lhi2c1, 0x39<<1, 0x80, I2C_MEMADD_SIZE_8BIT, &ENABLE, 1, 10000) != HAL_OK)
	{
		Error_Handler();
	}
	e_sprintf(msj4, "ENABLE: %d\n", ENABLE);
	printf(msj4);

	ENABLE &= ~0x01;
	if(HAL_I2C_Mem_Write(lhi2c1, 0x39<<1, 0x80, I2C_MEMADD_SIZE_8BIT, &ENABLE, 1, 10000) != HAL_OK)
	{
		Error_Handler();
	}

	if(HAL_I2C_Mem_Read(lhi2c1, 0x39<<1, 0x80, I2C_MEMADD_SIZE_8BIT, &ENABLE, 1, 10000) != HAL_OK)
	{
		Error_Handler();
	}
	e_sprintf(msj4, "ENABLE: %d\n", ENABLE);
	printf(msj4);
	printf("Disable AS7341\n");
}
