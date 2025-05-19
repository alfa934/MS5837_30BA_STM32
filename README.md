# MS5837_30BA_STM32
An MS5837 30BA Pressure Sensor library written in C for STM32Cube ide.

It is heavily based on the Blue Robotics MS5837 30BA Arduino library.

## Wires / Connection
- Red : 3V3
- Black : GND
- White : SDA
- GREEN : SCL

## Example (how-to-use-it)
1. Place `MS5837_lib.h` in the `Core/Inc/` directory
2. Place `MS5837_lib.c` in the `Core/Src/` directory
3. Code in `Core/Src/main.c` file
   
## Blocking Code Example
```C
/* USER CODE BEGIN Includes */
#include "MS5837_lib.h"
/* USER CODE END Includes */
```
```C
/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
```
```C
/* USER CODE BEGIN 0 */
MS5837_t press_sensor = {0};
/* USER CODE END 0 */
```
```C
/* USER CODE BEGIN 2 */
MS5837_Init(&hi2c1, &press_sensor, 50); //-- (i2c handle, MS5837 handle, delay_ms)
/* USER CODE END 2 */
```
```C
/* Infinite loop */
/* USER CODE BEGIN WHILE */
while (1)
{
  MS5832_Process(&hi2c1, &press_sensor);
  float pressure = press_sensor.pressure_mbar;
  float temperature = press_sensor.temperature;
  HAL_Delay(50);
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
}
/* USER CODE END 3 */
```

## Non-Blocking Code Example
```C
/* USER CODE BEGIN Includes */
#include "MS5837_lib.h"
/* USER CODE END Includes */
```
```C
/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim1; //--- configure to run every 1 ms
```
```C
/* USER CODE BEGIN 0 */
MS5837_t press_sensor = {0};
int pres_time = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  
  if(htim == &htim1) //--- general timer (run every 1 ms)
  {
    pres_time++;
    if(pres_time >= press_sensor.delay_ms)
    {
      MS5832_Process(&hi2c1, &press_sensor);
      float pressure = press_sensor.pressure_mbar;
      float temperature = press_sensor.temperature;
      pres_time = 0;
    }
  }
}

/* USER CODE END 0 */
```
```C
/* USER CODE BEGIN 2 */
MS5837_Init(&hi2c1, &press_sensor, 50); //-- (i2c handle, MS5837 handle, delay_ms)
/* USER CODE END 2 */
```
```C
/* Infinite loop */
/* USER CODE BEGIN WHILE */
while (1)
{

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
}
/* USER CODE END 3 */
```
