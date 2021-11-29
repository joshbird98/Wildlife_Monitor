/*
 * mcp41010.c
 *
 *  Created on: Nov 11, 2021
 *      Author: Josh
 *
 *	256 position digital potentiometer
 *  R = Rwiper + ((MAX_RESISTANCE * (POSITIONS - D)) / POSITIONS));
 *  pre-amp gain = - 200k / R
 wb tied together
 */

#include "mcp41010.h"

//NOT TESTED ON HARDWARE

//this function sets the digipot to the closest available resistance (res) value
//and returns the actual resistance the digipot achieved
float set_digipot(uint16_t res, SPI_HandleTypeDef hspi)
{
	uint8_t spi_data[2];
	spi_data[0] = (WRITE_CMD | POT_0);

	// correct for invalid resistance input
	if (res < 1) res = 1;
	if (res > (RMAX-1)) res = RMAX;

	spi_data[1] = STEPS - ((STEPS * (res - RWIPER)) / RMAX);
    if (spi_data[1] < 2) spi_data[1] = 255;	//correct for rollover

    //now send the data to digipot
    HAL_GPIO_WritePin(CS_MIC_GAIN_GPIO_Port, CS_MIC_GAIN_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi, spi_data, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_MIC_GAIN_GPIO_Port, CS_MIC_GAIN_Pin, GPIO_PIN_SET);

    float actual_resistance = RWIPER + (float)(RMAX * (STEPS - spi_data[1])) / (float)STEPS;
    return actual_resistance;
}

//this function sets the micropone gain to the closest available gain
//and returns the gain that it is actually set to
//here, gain can go from about 20 to 2000
float set_mic_gain(uint16_t gain, SPI_HandleTypeDef hspi)
{
	// gain = 200k/R, ie R = 200k/gain
	uint16_t resistance = (uint16_t)(200000.0/gain);
	float actual_resistance = set_digipot(resistance, hspi);
	return (200000.0 / actual_resistance);
}
