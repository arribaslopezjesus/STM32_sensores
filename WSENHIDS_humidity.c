/*
 * WSENHIDS_humidity.c
 *
 *  Created on: 10 ago. 2021
 *      Author: JAL
 */

#include "WSENHIDS_humidity.h"
#include "string.h"
#include "printf.h"
#include "../conf.h"
#include "math.h"
#include "Sim7600.h"


//Variables globales

uint8_t T1_T0 = 0;
uint8_t H0_T0_OUT_Reg[2] = {0};
uint8_t	H1_T0_OUT_Reg[2] = {0};
uint8_t Reg_Cal_0[4] = {0};
uint8_t Reg_Cal_2[4] = {0};

/**
 * @brief Función que detecta si el sensor tiene datos disponible actualmente. Funciona también cómo una espera flexible.
 * @param lhi2c1 Estructura que maneja los parametros necesarios para configurar y utilizar el i2c1
 * @return void
 */
void DATA_HIDS_DISP(I2C_HandleTypeDef *lhi2c1){
	uint8_t status;
	uint8_t i = 0;
	do{
		if(HAL_I2C_Mem_Read(lhi2c1, WSEN_HIDS_I2C_ADDRESS, 0x27, I2C_MEMADD_SIZE_8BIT, &status, 1, 10000) != HAL_OK)
		{
				Error_Handler();
		}
		i++;
		HAL_Delay(200);
		if(i > 5)
			return;
	}while((status != 3));
}

/**
 * @brief Función que detecta e inicializa el sensor WSEN_HIDS.
 * @param lhi2c1 Estructura que maneja los parametros necesarios para configurar y utilizar el i2c1
 * @return booleano 1:Inicialización correcta 0:Inicialización incorrecta
 */
uint8_t INIT_WSENHIDS(I2C_HandleTypeDef *lhi2c1)
{
	uint8_t ID_HUM_WSIN = 0;
	uint8_t CTR1 = 0;
	uint8_t CTR2 = 0;

	//Lectura del ID del sensor. Comprobación de su disponibilidad.
	if(HAL_I2C_Mem_Read(lhi2c1, WSEN_HIDS_I2C_ADDRESS, 0x0F, I2C_MEMADD_SIZE_8BIT, &ID_HUM_WSIN, 1, 10000) != HAL_OK)
	{
			debug_serie("[WSENHIDS.c] WSENHIDS no encontrado\n");
			return 0;
			Error_Handler();
	}

	//Configuración del registro CTR1
	if(HAL_I2C_Mem_Read(lhi2c1, WSEN_HIDS_I2C_ADDRESS, 0x20, I2C_MEMADD_SIZE_8BIT, &CTR1, 1, 10000) != HAL_OK)
	{
			Error_Handler();
	}

	CTR1 |= 0x04;       //BDU = 1
	CTR1 |= (1<<7);     //PD = 1
	CTR1 &= ~(1<<0) & ~(1<<1); //ODR = 00
	
	if(HAL_I2C_Mem_Write(lhi2c1, WSEN_HIDS_I2C_ADDRESS, 0x20, I2C_MEMADD_SIZE_8BIT, &CTR1, 1, 10000) != HAL_OK)
	{
			Error_Handler();
	}

	//Lectura de parámetros de calibración del sensor
	if(HAL_I2C_Mem_Read(lhi2c1, WSEN_HIDS_I2C_ADDRESS, 0x35, I2C_MEMADD_SIZE_8BIT, &T1_T0, 1, 10000) != HAL_OK)
	{
				Error_Handler();
	}

	if(HAL_I2C_Mem_Read(lhi2c1, WSEN_HIDS_I2C_ADDRESS, 0xB0, I2C_MEMADD_SIZE_8BIT, Reg_Cal_0, 4, 10000) != HAL_OK)
	{
				Error_Handler();
	}

	if(HAL_I2C_Mem_Read(lhi2c1, WSEN_HIDS_I2C_ADDRESS, 0xB6, I2C_MEMADD_SIZE_8BIT, H0_T0_OUT_Reg, 2, 10000) != HAL_OK)
	{
					Error_Handler();
	}

	if(HAL_I2C_Mem_Read(lhi2c1, WSEN_HIDS_I2C_ADDRESS, 0xBA, I2C_MEMADD_SIZE_8BIT, H1_T0_OUT_Reg, 2, 10000) != HAL_OK)
	{
						Error_Handler();
	}


	if(HAL_I2C_Mem_Read(lhi2c1, WSEN_HIDS_I2C_ADDRESS, 0xBC, I2C_MEMADD_SIZE_8BIT, Reg_Cal_2, 4, 10000) != HAL_OK)
	{
					Error_Handler();
	}

	//Configuración del registro CTR2
	if(HAL_I2C_Mem_Read(lhi2c1, WSEN_HIDS_I2C_ADDRESS, 0x21, I2C_MEMADD_SIZE_8BIT, &CTR2, 1, 10000) != HAL_OK)
	{
			Error_Handler();
	}
	CTR2 |= (1<<0);     //one_shot=1
	if(HAL_I2C_Mem_Write(lhi2c1, WSEN_HIDS_I2C_ADDRESS, 0x21, I2C_MEMADD_SIZE_8BIT, &CTR2, 1, 10000) != HAL_OK)
	{
			Error_Handler();
	}

	return 1;
}

/**
 * @brief Función que lee y calcula los datos de humedad y temperatura.
 * @param lhi2c1 Estructura que maneja los parametros necesarios para configurar y utilizar el i2c1
 *        datosStruct Estrcutura que contine los diferentes datos de los sensores medioambientales de la estación
 * @return void
 */
void HUMTEMP_WSENHIDS(I2C_HandleTypeDef *lhi2c1, param_sens *datosStruct)
{
	uint16_t tmp = 0;
	uint16_t buffer = 0;
	uint16_t x = 0;
	uint16_t y = 0;
	uint16_t T0_degC_x8 = 0;
	uint16_t T1_degC_x8 = 0;
	uint16_t T0_degC = 0;
	uint16_t T1_degC = 0;
	uint16_t T0_OUT = 0;
	uint16_t T1_OUT = 0;
	uint16_t T_OUT = 0;
	uint8_t HIDS_OUT_REG[4]= {0};
	uint8_t H0_rH, H1_rH = 0;
	uint16_t H0_T0_OUT= 0;
	uint16_t H1_T0_OUT = 0;
	uint16_t H_T_OUT = 0;
	float t_temp = 0.0;
	float deg = 0.0;
	char Temperatura[10] = {0};
	float h_temp = 0.0;
	float hum = 0.0;
	char Humidity[10] = {0};
	char msj2[100];

	//Lectura de los datos brutos del sensor
	if(HAL_I2C_Mem_Read(lhi2c1, WSEN_HIDS_I2C_ADDRESS, 0xA8, I2C_MEMADD_SIZE_8BIT, HIDS_OUT_REG, 4, 10000) != HAL_OK)
	{
			Error_Handler();
	}

	//Calculo de la humedad
	H0_rH = Reg_Cal_0[0]/2;
	H1_rH = Reg_Cal_0[1]/2;
	H0_T0_OUT = (H0_T0_OUT_Reg[1]<<8) + H0_T0_OUT_Reg[0];
	H1_T0_OUT = (H1_T0_OUT_Reg[1]<<8) + H1_T0_OUT_Reg[0];
	H_T_OUT = (HIDS_OUT_REG[1]<<8) + HIDS_OUT_REG[0];
	hum = H1_rH - H0_rH;
	h_temp = (float)(((int16_t)H_T_OUT - (int16_t)H0_T0_OUT) * hum) / (float)((int16_t)H1_T0_OUT - (int16_t)H0_T0_OUT);
	hum = (float)H0_rH;
	datosStruct->humedad = hum + h_temp; // provide signed % measurement unit

	//Calculo de la temperatura
	tmp = T1_T0;
	buffer = Reg_Cal_0[2];
	x = ((tmp & 0x03)<<8);
	T0_degC_x8 = x | buffer;
	T0_degC = (T0_degC_x8)/8;
	buffer = Reg_Cal_0[3];
	y = ((tmp & 0x0C)<<6);
	T1_degC_x8 = y | buffer;
	T1_degC = T1_degC_x8/8;
	T0_OUT = (Reg_Cal_2[1]<<8) + Reg_Cal_2[0];
	T1_OUT = (Reg_Cal_2[3]<<8) + Reg_Cal_2[2];
	T_OUT = (HIDS_OUT_REG[3]<<8) + HIDS_OUT_REG[2];
	deg=(float)(int16_t)(T1_degC) - (int16_t)(T0_degC);
	t_temp = (float) (((int16_t)T_OUT - (int16_t)T0_OUT) * deg) / (float)((int16_t)T1_OUT - (int16_t)T0_OUT);
	deg = (float)((int16_t)T0_degC);
	datosStruct->temp_hids = deg + t_temp;
	myFtoa(datosStruct->temp_hids, Temperatura);
	myFtoa(datosStruct->humedad, Humidity);
	e_sprintf(msj2, "[WSEN_HIDS.c] Temperatura:%s Humedad:%s %%\n\n", Temperatura, Humidity);
	debug_serie(msj2);

}

/**
 * @brief Función que desactiva el sensor WSEN_HIDS
 * @param lhi2c1 Estructura que maneja los parametros necesarios para configurar y utilizar el i2c1
 * @return void
 */
void OFF_WSENHIDS(I2C_HandleTypeDef *lhi2c1)
{
	uint8_t CTR1 = 0;

	if(HAL_I2C_Mem_Read(lhi2c1, WSEN_HIDS_I2C_ADDRESS, 0x20, I2C_MEMADD_SIZE_8BIT, &CTR1, 1, 10000) != HAL_OK)
	{
			Error_Handler();
	}

	CTR1 &= ~(1<<7);//PD = 1

	if(HAL_I2C_Mem_Write(lhi2c1, WSEN_HIDS_I2C_ADDRESS, 0x20, I2C_MEMADD_SIZE_8BIT, &CTR1, 1, 10000) != HAL_OK)
	{
			Error_Handler();
	}
}
