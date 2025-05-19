/*
 * MS5837_lib.h
 *
 *  Created on: May 18, 2025
 *      Author: alfa934
 */

//--- White=SDA, Green=SCL,

#ifndef INC_MS5837_LIB_H_
#define INC_MS5837_LIB_H_

#include "main.h"

/******************************************************************************
 * Defines
 *****************************************************************************/
#define MS5837_I2C_ADDR    (0x76 << 1)  // MS5837 I2C address
#define MS5837_RESET_CMD 	0x1E      	// Reset command
#define MS5837_PROM_READ 	0xA0      	// PROM read base address
#define MS5837_CONVERT_D1 	0x4A     	// Convert D1 command (pressure)
#define MS5837_CONVERT_D2 	0x5A     	// Convert D2 command (temperature)
#define MS5837_ADC_READ 	0x00       	// ADC read command


/******************************************************************************
 * Enums
 *****************************************************************************/
typedef enum
{
	START_CONVERT_D1,
	WAIT_CONVERT_D1,
	READ_ADC_D1,
	START_CONVERT_D2,
	WAIT_CONVERT_D2,
	READ_ADC_D2,
	CALCULATE_D1_D2

} MS5837_States ;


/******************************************************************************
 * Structs
 *****************************************************************************/
typedef struct
{
	uint8_t 		current_command;
	uint16_t 		prom_coefficients[8];
	uint16_t		delay_ms;
	uint32_t 		conversion_start_time;

	int32_t 		pressure_raw;
	int32_t 		temperature_raw;

	uint32_t 		pressure_D1;
	uint32_t 		temperature_D2;

	float 			pressure_mbar;
	float 			temperature_celsius;
	float 			fluid_density;

	MS5837_States 	state;

} MS5837_t ;


/******************************************************************************
 * Function Prototypes
 *****************************************************************************/
void 	MS5837_SetFluidDensity( MS5837_t *sensor, float density );
void 	MS5837_Init( I2C_HandleTypeDef *I2Cx, MS5837_t *sensor, uint16_t delay_ms );
void 	MS5837_StartConversion( I2C_HandleTypeDef *I2Cx, MS5837_t *sensor, uint8_t command );
void 	MS5837_ReadADC( I2C_HandleTypeDef *I2Cx, MS5837_t *sensor );
void 	MS5837_Calculation( MS5837_t *sensor );
void 	MS5832_Process( I2C_HandleTypeDef *I2Cx, MS5837_t *sensor);


#endif /* INC_MS5837_LIB_H_ */
