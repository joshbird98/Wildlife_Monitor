/*
 * RadioCommands.h
 *
 *  Created on: 5 Dec 2021
 *      Author: theag
 */

#ifndef INC_RADIOCOMMANDS_H_
#define INC_RADIOCOMMANDS_H_
#include "stm32l4xx_hal_spi.h"

#define SPIRead 0x00
#define SPIWrite 0x80
#define TRANSMITMODE  1
#define RECEIVEMODE 0
#define TRUE 1
#define FALSE 0
#include "main.h"

void waitBusy()
{
  while (HAL_GPIO_ReadPin(RADIO_BUSY_GPIO_Port,RADIO_BUSY_Pin) == 15);
}


//Generic SPI read/write command
uint8_t* SPItransferCmd(uint8_t cmd, uint8_t* dataOut, uint8_t* dataIn, uint8_t numBytes, SPI_HandleTypeDef hspi)
{
	waitBusy();
	HAL_GPIO_WritePin(CS_RADIO_GPIO_Port, CS_RADIO_Pin, GPIO_PIN_RESET);

	if ((cmd == SPIWrite) && (dataOut != NULL))
		HAL_SPI_Transmit(&hspi, dataOut, numBytes, 100);


	if ((cmd == SPIRead) && (dataIn != NULL))
		HAL_SPI_TransmitReceive(&hspi, dataOut, dataIn, numBytes, 100);

	//Serial.print(dataOut[0]);
	HAL_GPIO_WritePin(CS_RADIO_GPIO_Port, CS_RADIO_Pin, GPIO_PIN_SET);

	if (dataIn != NULL)
		return dataIn;
	return NULL;
}


void setStandby(SPI_HandleTypeDef hspi)
{
  uint8_t data[] = {0x80, 0x00};
  SPItransferCmd(SPIWrite, data, NULL, 2, hspi);
}

void setBufferbase(SPI_HandleTypeDef hspi)
{
  uint8_t data[] = {0x8F, 0x00, 0x00};
  SPItransferCmd(SPIWrite, data, NULL, 3, hspi);
}

void setPackettype(SPI_HandleTypeDef hspi)
{
  uint8_t data[] = {0x8A, 0x01};
  SPItransferCmd(SPIWrite, data, NULL, 2, hspi);
}

void setFallback(SPI_HandleTypeDef hspi)
{
  uint8_t data[] = {0x93, 0x20};
  SPItransferCmd(SPIWrite, data, NULL, 2, hspi);
}

void setCADParam(SPI_HandleTypeDef hspi)
{
  uint8_t data[] = {0x88, 0x03, 0x16, 0x0A, 0x00, 0x00, 0x00, 0x00};
  SPItransferCmd(SPIWrite, data, NULL, 8, hspi);
}

void clrIRQ(SPI_HandleTypeDef hspi)
{
  uint8_t data[] = {0x02, 0x03, 0xFF};
  SPItransferCmd(SPIWrite, data, NULL, 3, hspi);
}

void setDIO(SPI_HandleTypeDef hspi)
{
  uint8_t data[] = {0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  SPItransferCmd(SPIWrite, data, NULL, 9, hspi);
}

void setDIORead(SPI_HandleTypeDef hspi)
{
  uint8_t data[] = {0x08, 0x02, 0x62, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00};
  SPItransferCmd(SPIWrite, data, NULL, 9, hspi);
}

void setDIOTransmit(SPI_HandleTypeDef hspi)
{
  uint8_t data[] = {0x08, 0x02, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
  SPItransferCmd(SPIWrite, data, NULL, 9, hspi);
}

void calibrate(SPI_HandleTypeDef hspi)
{
  uint8_t data[] = {0x89, 0x7F};
  SPItransferCmd(SPIWrite, data, NULL, 2, hspi);
}

void getDevErrors(uint8_t *returnData, SPI_HandleTypeDef hspi)
{
  uint8_t data[] = {0x17, 0x00, 0x00, 0x00};
  SPItransferCmd(SPIRead, data, returnData, 4, hspi);
}

void setDIO3(SPI_HandleTypeDef hspi)
{
  uint8_t data[] = {0x97, 0x00, 0x00, 0x01, 0x40};
  SPItransferCmd(SPIWrite, data, NULL, 5, hspi);
}

void setmodParam(SPI_HandleTypeDef hspi)
{
  uint8_t data[] = {0x8B, 0x09, 0x04, 0x03, 0x00};
  SPItransferCmd(SPIWrite, data, NULL, 5, hspi);
}

void setSyncword(SPI_HandleTypeDef hspi)
{
  uint8_t data[] = {0x0D, 0x07, 0x40, 0x14, 0x24};
  SPItransferCmd(SPIWrite, data, NULL, 5, hspi);
}

void setReadIQpol(uint8_t *returnData, SPI_HandleTypeDef hspi)
{
  uint8_t data[] = {0x1D, 0x07, 0x36, 0x00, 0x00};
  SPItransferCmd(SPIRead, data, returnData, 5, hspi);
}


void fixIQ(SPI_HandleTypeDef hspi)
{
  uint8_t data[] = {0x0D, 0x07, 0x36, 0x09};
  SPItransferCmd(SPIWrite, data, NULL, 4, hspi);
}

void setPacketparam(SPI_HandleTypeDef hspi)
{
  uint8_t data[] = {0x8C, 0x00, 0x08, 0x00, 0xFF, 0x01, 0x00};
  SPItransferCmd(SPIWrite, data, NULL, 7, hspi);
}


void SetOCP(SPI_HandleTypeDef hspi)
{
  uint8_t data[] = {0x0D, 0x08, 0xE7, 0x18};
  SPItransferCmd(SPIWrite, data, NULL, 4, hspi);
}

void setDIO2(SPI_HandleTypeDef hspi)
{
  uint8_t data[] = {0x9D, 0x01};
  SPItransferCmd(SPIWrite, data, NULL, 2, hspi);
}

void regParam(SPI_HandleTypeDef hspi)
{
  uint8_t data[] = {0x96, 0x01};
  SPItransferCmd(SPIWrite, data, NULL, 2, hspi);
}


void calibrateImage(SPI_HandleTypeDef hspi)
{
  uint8_t data[] = {0x98, 0x6B, 0x6F};
  SPItransferCmd(SPIWrite, data, NULL, 3, hspi);
}

void setRFfreq(SPI_HandleTypeDef hspi)
{
  uint8_t data[] = {0x86, 0x1B, 0x20, 0x00, 0x00};
  SPItransferCmd(SPIWrite, data, NULL, 5, hspi);
}

void getOCP(SPI_HandleTypeDef hspi)
{
  uint8_t data[] = {0x1D, 0x08, 0xE7, 0x00, 0x00};
  SPItransferCmd(SPIRead, data, NULL, 5, hspi);
}

void SetPaconfig(SPI_HandleTypeDef hspi)
{
  uint8_t data[] = {0x95, 0x04, 0x07, 0x00, 0x01};
  SPItransferCmd(SPIWrite, data, NULL, 5, hspi);
}

void SetTxparameters(SPI_HandleTypeDef hspi)
{
  uint8_t data[] = {0x8E, 0x0A, 0x04};
  SPItransferCmd(SPIWrite, data, NULL, 3, hspi);
}

void getPacket(SPI_HandleTypeDef hspi)
{
  uint8_t packettype[2];
  uint8_t data[] = {0x11, 0x00, 0x00};
  SPItransferCmd(SPIRead, data, packettype, 3, hspi);
  /*print("Packet Type:\t\t");
  if (packettype[1]==(uint8_t)1)
    println("LoRa");
  else
    println("Other");
	*/
}

void getBufferstat(uint8_t* bffr, SPI_HandleTypeDef hspi)
{
  uint8_t data[] = {0x13, 0x00, 0x00};
  SPItransferCmd(SPIRead, data, bffr, 3, hspi);
  bffr[1] -= 1;
}

void readBuffer(uint8_t *recMessage, uint8_t len, uint8_t start, SPI_HandleTypeDef hspi)
{
  waitBusy();
  HAL_GPIO_WritePin(CS_RADIO_GPIO_Port, CS_RADIO_Pin, GPIO_PIN_RESET);
  uint8_t data[] = {0x1E, start, 0x00};
  HAL_SPI_Transmit(&hspi, data, 3, 100);
  HAL_SPI_Receive(&hspi, recMessage, len, 100);
  HAL_GPIO_WritePin(CS_RADIO_GPIO_Port, CS_RADIO_Pin, GPIO_PIN_SET);	//modified from RESET
}

void writePayload(const char* payload, SPI_HandleTypeDef hspi)
{
  uint8_t data[sizeof payload + 2];
  data[0] = 0x0E;
  data[1] = 0x00;
  for(uint8_t i = 0; i < sizeof payload; i++)
    data[i+2] = payload[i];
  SPItransferCmd(SPIWrite,data,NULL,3, hspi);
}

void Radiobegin(SPI_HandleTypeDef hspi)
{
  setStandby(hspi);
  setBufferbase(hspi);
  setPackettype(hspi);
  setFallback(hspi);
  setCADParam(hspi);
  clrIRQ(hspi);
  setDIO(hspi);
  calibrate(hspi);
  setStandby(hspi);
  setDIO3(hspi);
  setmodParam(hspi);
  setSyncword(hspi);
  setPacketparam(hspi);
  SetOCP(hspi);
  setDIO2(hspi);
  regParam(hspi);
  calibrateImage(hspi);
  setRFfreq(hspi);
  SetPaconfig(hspi);
  SetTxparameters(hspi);
}

#endif /* INC_RADIOCOMMANDS_H_ */
