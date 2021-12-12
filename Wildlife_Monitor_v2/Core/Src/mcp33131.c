/*
 * mcp33131.c
 *
 *  Created on: Nov 11, 2021
 *      Author: Josh
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "mcp33131.h"

// Gets 16 bit sample from the MCP33131 ADC
uint16_t mcp33131_get16b(SPI_HandleTypeDef hspi1)
{
	uint8_t buffer[2];

	//set MOSI to high for duration of function, as required
	uint8_t dummy [2] = {0xFF, 0xFF};
	HAL_SPI_Transmit(&hspi1, dummy, 1, 10);

	//trigger conversion and wait until complete
	HAL_GPIO_WritePin(CS_MIC_ADC_GPIO_Port, CS_MIC_ADC_Pin, GPIO_PIN_SET);
	uint16_t start_tim_val = TIM17->CNT;
	while (start_tim_val != TIM17->CNT);//wait for timer 17 to loop (>750ns, ie >conversion time)
	HAL_GPIO_WritePin(CS_MIC_ADC_GPIO_Port, CS_MIC_ADC_Pin, GPIO_PIN_RESET);

	//receive 2 byte sample, keeping MOSI high as required
    HAL_SPI_TransmitReceive (&hspi1, dummy, buffer, 2, 10);
    uint16_t sample = ((uint16_t)buffer[0] << 8 ) | ((uint16_t) buffer[1]);

	return sample;
}

// Calibrates the MCP33131 ADC
// Similar to taking a sample, except the clock just keeps going until 1024 cycles complete
// these 1024 clock cycles triggers a calibration to begin, which then takes 650ms
void mcp33131_calibrate(SPI_HandleTypeDef hspi1)
{
	uint8_t buffer[128];

	//set MOSI to high for duration of function, as required
	uint8_t dummy[128] = { [0 ... 127] = 0xFF };
	HAL_SPI_Transmit(&hspi1, dummy, 1, 10);

	//trigger conversion and wait until complete
	HAL_GPIO_WritePin(CS_MIC_ADC_GPIO_Port, CS_MIC_ADC_Pin, GPIO_PIN_SET);
	uint16_t start_tim_val = TIM17->CNT;
	while (start_tim_val != TIM17->CNT);//wait for timer 17 to loop (>750ns, ie >conversion time)
	HAL_GPIO_WritePin(CS_MIC_ADC_GPIO_Port, CS_MIC_ADC_Pin, GPIO_PIN_RESET);

	//clock needs to complete 1024 cycles ie 128 bytes of cycles.
	//so send 128 bytes of 0xFF, with the nonsense received just going into a useless buffer
	HAL_SPI_TransmitReceive (&hspi1, dummy, buffer, 128, 10);
	HAL_Delay(650);	//wait for calibration to complete, yes this is blocking but oh well
}

#ifdef __cplusplus
}
#endif
