/*
 * WSENPADS_pressure.c
 *
 *  Created on: 10 ago. 2021
 *      Author: JAL
 */
#include "WSENPADS_pressure.h"

#include "string.h"
#include "printf.h"
#include "../conf.h"
#include "math.h"

#include "Sim7600.h"

/**
 * @brief Función que detecta si el sensor tiene datos disponible actualmente. Funciona también cómo una espera flexible.
 * @param lhi2c1 Estructura que maneja los parametros necesarios para configurar y utilizar el i2c1
 * @return void
 */
void DATA_PADS_DISP(I2C_HandleTypeDef *lhi2c1, uint8_t I2C_Adress){
	uint8_t status;
	uint8_t i = 0;
	do{
		if(HAL_I2C_Mem_Read(lhi2c1, I2C_Adress, 0x27, I2C_MEMADD_SIZE_8BIT, &status, 1, 10000) != HAL_OK)
		{
				Error_Handler();
		}
		i++;
		HAL_Delay(200);
		if(i > 5)
			return;
	}while((status & 0x03) != 0x03);
}

/**
 * @brief Función que detecta e inicializa el sensor WSEN_PADS.
 * @param lhi2c1 Estructura que maneja los parametros necesarios para configurar y utilizar el i2c1
 * @return booleano 1:Inicialización correcta 0:Inicialización incorrecta
 */
uint8_t INIT_WSENPADS(I2C_HandleTypeDef *lhi2c1, uint8_t I2C_Adress)
{
	uint8_t ID_PADS_PRESION = 0;
	uint8_t CTRL_2 = 0;
	uint8_t CTRL_1 = 0;

	//Lectura del ID del sensor. Comprobación de su disponibilidad.
	if(HAL_I2C_Mem_Read(lhi2c1, I2C_Adress, 0x0F, I2C_MEMADD_SIZE_8BIT, &ID_PADS_PRESION, 1, 10000) != HAL_OK)
	{
			debug_serie("[WSENPADS.c] WSENPADS no encontrado\n");
			return 0;
			Error_Handler();
	}

	//Configuración del registro CTRL_1.
	if(HAL_I2C_Mem_Read(lhi2c1, I2C_Adress, 0x10, I2C_MEMADD_SIZE_8BIT, &CTRL_1, 1, 10000) != HAL_OK)
	{
			Error_Handler();
	}

	CTRL_1 &= ~(1<<4) & ~(1<<5) & ~(1<<6); //ODR=000 PowerOFF/single-conversion-mode
	CTRL_1 |= 1<<1;  //BDU = 1

	if(HAL_I2C_Mem_Write(lhi2c1, I2C_Adress, 0x10, I2C_MEMADD_SIZE_8BIT, &CTRL_1, 1, 10000) != HAL_OK)
	{
			Error_Handler();
	}

	//Configuración del registro CTRL_2.
	if(HAL_I2C_Mem_Read(lhi2c1, I2C_Adress, 0x11, I2C_MEMADD_SIZE_8BIT, &CTRL_2, 1, 10000) != HAL_OK)
	{
			Error_Handler();
	}

	CTRL_2 &= ~(1<<1);   //low-power mode

	if(HAL_I2C_Mem_Write(lhi2c1, I2C_Adress, 0x11, I2C_MEMADD_SIZE_8BIT, &CTRL_2, 1, 10000) != HAL_OK)
	{
			Error_Handler();
	}

	if(HAL_I2C_Mem_Read(lhi2c1, I2C_Adress, 0x11, I2C_MEMADD_SIZE_8BIT, &CTRL_2, 1, 10000) != HAL_OK)
	{
			Error_Handler();
	}

	CTRL_2 |= 1<<0; //One shoot

	if(HAL_I2C_Mem_Write(lhi2c1, I2C_Adress, 0x11, I2C_MEMADD_SIZE_8BIT, &CTRL_2, 1, 10000) != HAL_OK)
	{
			Error_Handler();
	}

	return 1;

}

/**
 * @brief Función que lee y calcula los datos de presión y temperatura.
 * @param lhi2c1 Estructura que maneja los parametros necesarios para configurar y utilizar el i2c1
 *        datosStruct Estrcutura que contine los diferentes datos de los sensores medioambientales de la estación
 *        I2C_Adress DIreccion I2c del sensor WSEN_PADS.
 * @return void
 */
void PRESTEMP_WSENPADS(I2C_HandleTypeDef *lhi2c1, param_sens *DatosStruct, uint8_t I2C_Adress)
{
	uint8_t Data_T_P[5] = {0};
	uint32_t Presion;
	uint16_t Temp;
	int16_t T_neg;
	char Pres[10] = {0};
	char T[10] = {0};
	char msj[50];
	//Lectura de temperatura y presion
	if(HAL_I2C_Mem_Read(lhi2c1, I2C_Adress, 0x28, I2C_MEMADD_SIZE_8BIT, Data_T_P, 5, 10000) != HAL_OK)
	{
			Error_Handler();
	}

	//Cálculo de temperatura y presión
	Presion= (Data_T_P[2]<<16) + (Data_T_P[1]<<8) + Data_T_P[0];
	DatosStruct->presion = Presion/40960;
	Temp = (Data_T_P[4]<<8) + Data_T_P[3];
	if(Temp > 32768)
	{
		T_neg = Temp - 32768;
		DatosStruct->temp_pads = (double)(0 - 32768 + T_neg)/100;
	}
	else
	{
		DatosStruct->temp_pads = (double)Temp/100;
	}
	myFtoa(DatosStruct->temp_pads, T);
	myFtoa(DatosStruct->presion, Pres);
	e_sprintf(msj, "[WSENPADS.c] Presion:%s kPa", Pres);
	debug_serie(msj);
	e_sprintf(msj, "[WSENPADS.c] Temperatura:%s\n\n", T);
	debug_serie(msj);


}

/**
 * @brief Función que desactiva el sensor WSEN_PADS
 * @param lhi2c1 Estructura que maneja los parametros necesarios para configurar y utilizar el i2c1
 * @return void
 */
void OFF_WSENPADS(I2C_HandleTypeDef *lhi2c1, uint8_t I2C_Adress)
{
	uint8_t CTRL_1 = 0;

	if(HAL_I2C_Mem_Read(lhi2c1, I2C_Adress, 0x10, I2C_MEMADD_SIZE_8BIT, &CTRL_1, 1, 10000) != HAL_OK)
	{
			Error_Handler();
	}

	CTRL_1 &= ~(1<<4) & ~(1<<5) & ~(1<<6); //ODR=000

	if(HAL_I2C_Mem_Write(lhi2c1, I2C_Adress, 0x10, I2C_MEMADD_SIZE_8BIT, &CTRL_1, 1, 10000) != HAL_OK)
	{
			Error_Handler();
	}

}
