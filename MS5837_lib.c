/*
 * MS5837_lib.c
 *
 *  Created on: May 18, 2025
 *      Author: ALFA
 */

#include "MS5837_lib.h"


void MS5837_SetFluidDensity( MS5837_t *sensor, float density )
{
	sensor -> fluid_density = density;

	return;
}


void MS5837_Init( I2C_HandleTypeDef *I2Cx, MS5837_t *sensor, uint16_t delay_ms )
{
	//--- minimum time 40 ms
	sensor -> delay_ms = delay_ms - 1; //--- Offset 1, since counter starts at 0

    uint8_t cmd = MS5837_RESET_CMD;

    HAL_I2C_Master_Transmit(I2Cx, MS5837_I2C_ADDR, &cmd, 1, HAL_MAX_DELAY);

    HAL_Delay(10);

    uint8_t data[2];

    for(int i = 0; i < 8; i++)
    {
        cmd = MS5837_PROM_READ + (i * 2);

        HAL_I2C_Master_Transmit(I2Cx, MS5837_I2C_ADDR, &cmd, 1, HAL_MAX_DELAY);
        HAL_I2C_Master_Receive (I2Cx, MS5837_I2C_ADDR, data, 2, HAL_MAX_DELAY);

        sensor -> prom_coefficients[i] = (data[0] << 8) | data[1];
    }

    return;
}


void MS5837_StartConversion( I2C_HandleTypeDef *I2Cx, MS5837_t *sensor, uint8_t command )
{
	HAL_StatusTypeDef status;

    sensor -> current_command = command;
    sensor -> conversion_start_time = HAL_GetTick();

    status = HAL_I2C_Master_Transmit(I2Cx, MS5837_I2C_ADDR, &command, 1, HAL_MAX_DELAY);
    if (status != HAL_OK)
    {
        sensor -> state = START_CONVERT_D1;
        return;
    }

    return;
}


void MS5837_ReadADC( I2C_HandleTypeDef *I2Cx, MS5837_t *sensor )
{
    uint8_t cmd = MS5837_ADC_READ;
    uint8_t data[3];
    HAL_StatusTypeDef status;

    status = HAL_I2C_Master_Transmit(I2Cx, MS5837_I2C_ADDR, &cmd, 1, HAL_MAX_DELAY);
    if (status != HAL_OK)
    {
        sensor -> state = START_CONVERT_D1;
        return;
    }

    status = HAL_I2C_Master_Receive(I2Cx, MS5837_I2C_ADDR, data, 3, HAL_MAX_DELAY);
    if (status != HAL_OK)
    {
        sensor -> state = START_CONVERT_D1;
        return;
    }

    uint32_t adc_value = (data[0] << 16) | (data[1] << 8) | data[2];

    if (sensor -> current_command == MS5837_CONVERT_D1)
    {
    	sensor -> pressure_D1 = adc_value;
    }
    else if (sensor -> current_command == MS5837_CONVERT_D2)
    {
    	sensor -> temperature_D2 = adc_value;
    }

    return;
}


void MS5837_Calculation(MS5837_t *sensor)
{   // Based on Blue Robotics MS5837 Arduino Library
	// Given C1-C6 and D1, D2, calculated TEMP and P
	// Do conversion first and then second order temp compensation
	int32_t dT 		= 0;
	int64_t SENS 	= 0;
	int64_t OFF 	= 0;
	int32_t SENSi 	= 0;
	int32_t OFFi 	= 0;
	int32_t Ti 		= 0;
	int64_t OFF2 	= 0;
	int64_t SENS2 	= 0;

	//--- Terms called
	dT = sensor -> temperature_D2 - (uint32_t)(sensor -> prom_coefficients[5]) * 256;

	SENS = (int64_t)(sensor -> prom_coefficients[1]) * 32768 + ((int64_t)(sensor -> prom_coefficients[3])*dT) / 256;
	OFF  = (int64_t)(sensor -> prom_coefficients[2]) * 65536 + ((int64_t)(sensor -> prom_coefficients[4])*dT) / 128;
	sensor -> pressure_raw = (sensor -> pressure_D1 * SENS / (2097152) - OFF) / (8192);

	//--- Temp conversion
	sensor -> temperature_raw = 2000 + (int64_t)(dT) * sensor -> prom_coefficients[6] / 8388608;

	//--- Second order compensation
	if((sensor -> temperature_raw / 100) < 20) 			//--- Low temp
	{
		Ti 		= (3 * (int64_t)(dT) * (int64_t)(dT)) / (8589934592);
		OFFi 	= (3 * (sensor -> temperature_raw - 2000) * (sensor -> temperature_raw - 2000)) / 2;
		SENSi 	= (5 * (sensor -> temperature_raw - 2000) * (sensor -> temperature_raw - 2000)) / 8;

		if((sensor -> temperature_raw / 100) < -15) 	//--- Very low temp
		{
			OFFi  = OFFi + 7 * (sensor -> temperature_raw + 1500) * (sensor -> temperature_raw + 1500);
			SENSi = SENSi + 4 * (sensor -> temperature_raw + 1500) * (sensor -> temperature_raw + 1500);
		}
	}
	else if((sensor -> temperature_raw / 100) >= 20) 	//--- High temp
	{
		Ti 		= 2 * (dT * dT) / (137438953472);
		OFFi 	= (1 * (sensor -> temperature_raw - 2000) * (sensor -> temperature_raw - 2000)) / 16;
		SENSi 	= 0;
	}

	//--- Calculate pressure and temp second order
	OFF2  = OFF  - OFFi;
	SENS2 = SENS - SENSi;

	sensor -> temperature_raw 		= (sensor -> temperature_raw - Ti);
	sensor -> pressure_raw 	  		= (((sensor -> pressure_D1 * SENS2) / 2097152 - OFF2) / 8192);

	sensor -> temperature_celsius 	= sensor -> temperature_raw / 100.0;
	sensor -> pressure_mbar   		= sensor -> pressure_raw / 10.0;

	return;
}


void MS5832_Process( I2C_HandleTypeDef *I2Cx, MS5837_t *sensor)
{
	uint16_t ADC_TIMEOUT = sensor -> delay_ms * 2;

	switch (sensor -> state)
	{
		case START_CONVERT_D1:
			MS5837_StartConversion(I2Cx, sensor, MS5837_CONVERT_D1);
			sensor -> state++;
			break;

		case WAIT_CONVERT_D1:
			if(HAL_GetTick() - sensor -> conversion_start_time >= 19)
			{
				sensor -> state++;
			}
			break;

		case READ_ADC_D1:
			MS5837_ReadADC(I2Cx, sensor);
			if(sensor -> pressure_D1 != 0)
			{
				sensor -> state++;
			}
			else if(HAL_GetTick() - sensor -> conversion_start_time > ADC_TIMEOUT)
			{
				sensor -> state = START_CONVERT_D1;
			}
			break;

		case START_CONVERT_D2:
			MS5837_StartConversion(I2Cx, sensor, MS5837_CONVERT_D2);
			sensor -> state++;
			break;

		case WAIT_CONVERT_D2:
			if(HAL_GetTick() - sensor -> conversion_start_time >= 19)
			{
				sensor -> state++;
			}
			break;

		case READ_ADC_D2:
			MS5837_ReadADC(I2Cx, sensor);
			if(sensor -> temperature_D2 != 0)
			{
				sensor -> state++;
			}
			else if(HAL_GetTick() - sensor -> conversion_start_time > ADC_TIMEOUT)
			{
				sensor -> state = START_CONVERT_D1;
			}
			break;

		case CALCULATE_D1_D2:
			MS5837_Calculation(sensor);
			sensor -> state = START_CONVERT_D1;
			break;
	}
}
