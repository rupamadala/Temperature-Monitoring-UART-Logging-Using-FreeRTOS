/*
 * dht11_driver.h
 *
 *  Created on: Nov 28, 2025
 *      Author: Rupa
 */

#ifndef DHT11_DRIVER_INC_DHT11_DRIVER_H_
#define DHT11_DRIVER_INC_DHT11_DRIVER_H_

/* USER CODE BEGIN Includes */
/* Private includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
/* USER CODE END */


/* USER CODE BEGIN */
/* Private Defines ------------------------------------------------------------------*/
#define DHT11_PORT 	GPIOA
#define DHT11_PIN 	GPIO_PIN_5
/* USER CODE END */


/* USER CODE BEGIN */
/* Private global variables ------------------------------------------------------------------*/
extern GPIO_InitTypeDef DHT11_GPIO_InitStruct;

typedef struct{
	uint8_t humidity_data;
	uint8_t humidity_dec_data;
	uint8_t temp_data;
	uint8_t temp_dec_data;
	uint8_t crc;
}DHT_D11_Data;
/* USER CODE END */


/* USER CODE BEGIN */
/* Private function prototypes ------------------------------------------------------------------*/
void DHT11_Init();
void DHT11_set_pinout();
void DHT11_set_pinin();
void DHT11_Start();
void Read_DHT11(DHT_D11_Data *sensor_data);
uint8_t crc_check(uint8_t *data);
void dwt_init(void);
void Delay_us(uint32_t delay);
/* USER CODE END */

#endif /* DHT11_DRIVER_INC_DHT11_DRIVER_H_ */
