/*
 * mcp33131.h
 *
 *  Created on: Nov 12, 2021
 *      Author: Josh
 */

#ifndef INC_MCP33131_H_
#define INC_MCP33131_H_

#include "main.h" 	// gives access to pin defines and HAL SPI

uint16_t mcp33131_get16b(SPI_HandleTypeDef hspi1);

#endif /* INC_MCP33131_H_ */
