/*
 * dht11_driver.c
 *
 *  Created on: Nov 28, 2025
 *      Author: Rupa
 */
#include "dht11_driver.h"


/* USER CODE BEGIN */
GPIO_InitTypeDef DHT11_GPIO_InitStruct = {0};
void dwt_init(){
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; //Enable Trace
	DWT->CYCCNT = 0;
	DWT->CTRL = DWT_CTRL_CYCCNTENA_Msk;
}

void Delay_us(uint32_t delay){
	uint32_t clock_cycles = (delay * SystemCoreClock)/1000000;
	uint32_t start = DWT->CYCCNT;
	while((DWT->CYCCNT - start)<=clock_cycles);
}

void DHT11_Init(){
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();

	// Configure DHT11 GPIO pin (just like MX_GPIO_Init())

	DHT11_GPIO_InitStruct.Pin	= DHT11_PIN;
	DHT11_GPIO_InitStruct.Mode 	= GPIO_MODE_OUTPUT_PP;
	DHT11_GPIO_InitStruct.Pull	= GPIO_NOPULL; //because internal pullup is too weak it should have external pullup with resistor 5kohms
	DHT11_GPIO_InitStruct.Speed	= GPIO_SPEED_FREQ_LOW;

	HAL_GPIO_Init(DHT11_PORT, &DHT11_GPIO_InitStruct);
	HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET); //Writing Pin to high

}

void DHT11_set_pinout(){
	DHT11_GPIO_InitStruct.Mode 	= GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(DHT11_PORT, &DHT11_GPIO_InitStruct);
}

void DHT11_set_pinin(){
	DHT11_GPIO_InitStruct.Mode 	= GPIO_MODE_INPUT;
	HAL_GPIO_Init(DHT11_PORT, &DHT11_GPIO_InitStruct);
}

void DHT11_Start(){
	DHT11_set_pinout();
	//Initially start signal to be sent to sensor
	HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_RESET);
	//wait for 20ms
	Delay_us(20000);
	//Write high for 20-40us
	HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
	Delay_us(30);
	//configured to input pin
	DHT11_set_pinin();
	//Check whether pin response started
	while((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) == GPIO_PIN_SET);
	// 1. Wait for DHT11 to pull LOW (80us)
	while((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) == GPIO_PIN_RESET);
	// 2. Wait for DHT11 to pull HIGH (80us)
	while((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) == GPIO_PIN_SET);

}

void Read_DHT11(DHT_D11_Data *sensor_data){
	uint8_t data[5] = {0}; //data storage for 40-bits
	DHT11_Start();
	for(int i = 0; i<40;i++){
		uint32_t count=0;
		// 3. wait for bit start low
		while((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) == GPIO_PIN_SET);
		// 4. Wait for DHT11 start of first bit (50us)
		while((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) == GPIO_PIN_RESET);
		// 5. Wait entire data length
		while((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) == GPIO_PIN_SET){
			count++;
		}
		if(count > 40){ //bit0 --- count= 26-28us and bit1 ----count = 70us
			data[i/8] |= (1 << (7 - (i%8)));
		}
	}
	if(crc_check(&data[0]) == data[4]){
		sensor_data->humidity_data = data[0];
		sensor_data->humidity_dec_data = data[1];
		sensor_data->temp_data = data[2];
		sensor_data->temp_dec_data = data[3];
	}
	else{
		//Error in CRC
	}

}

uint8_t crc_check(uint8_t *data){
	uint8_t crc_value = 0;
	crc_value = data[0] + data[1] + data[2] + data[3];
	return crc_value;
}

/* USER CODE END */
