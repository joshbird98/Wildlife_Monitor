/*
 * mcp33131.c
 *
 *  Created on: Nov 11, 2021
 *      Author: Josh
 */

#include "mcp33131.h"

//NOT TESTED ON HARDWARE
// Gets 16 bits sample from the MCP33131 ADC
uint16_t mcp33131_get16b(SPI_HandleTypeDef hspi1)
{
	uint16_t sample;

	HAL_GPIO_WritePin(CS_MIC_ADC_GPIO_Port, CS_MIC_ADC_Pin, GPIO_PIN_SET);
	uint16_t start_tim_val = TIM17->CNT;
	while (start_tim_val != TIM17->CNT);//wait for timer 17 to loop (750ns, ie conversion time)
	HAL_GPIO_WritePin(CS_MIC_ADC_GPIO_Port, CS_MIC_ADC_Pin, GPIO_PIN_RESET);

	HAL_SPI_Receive(&hspi1, &sample, 2, 100);

	return sample;
}
