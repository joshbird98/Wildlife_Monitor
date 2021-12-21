/*
 * mcp41010.h
 *
 *  Created on: Nov 12, 2021
 *      Author: Josh
 *
 *  Usage example
 *	mic_gain = set_mic_gain(DEFAULT_MIC_GAIN, hspi1); // initialises the microphone gain to 2000
 */

#ifndef INC_MCP41010_H_
#define INC_MCP41010_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define WRITE_CMD 0x10
#define SHDN_CMD 0x20

#define POT_0 0x01
#define POT_1 0x02
#define BOTH_POTS 0x03

#define STEPS 256
#define RMAX 10000
#define RWIPER 50

float set_digipot(uint16_t res, SPI_HandleTypeDef hspi);
float set_mic_gain(uint16_t gain, SPI_HandleTypeDef hspi);

#ifdef __cplusplus
}
#endif

#endif /* INC_MCP41010_H_ */
