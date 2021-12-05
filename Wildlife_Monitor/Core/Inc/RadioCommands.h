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

void waitBusy(){
  while (HAL_GPIO_ReadPin(GPIOC,RADIO_BUSY_Pin)== 15)
  ;
}


//Generic SPI read/write command
uint8_t* SPItransferCmd(uint8_t cmd, uint8_t* dataOut, uint8_t* dataIn, uint8_t numBytes){
waitBusy();
HAL_GPIO_WritePin( GPIOB, CS_RADIO_Pin, GPIO_PIN_RESET);
if (cmd==SPIWrite && dataOut!=NULL)
	HAL_SPI_Transmit(&hspi1, dataOut, numBytes,100);


if (cmd==SPIRead && dataIn!=NULL){
	HAL_SPI_TransmitReceive(&hspi1, dataOut, dataIn, numBytes, 100);

}
//Serial.print(dataOut[0]);
HAL_GPIO_WritePin( GPIOB, CS_RADIO_Pin, GPIO_PIN_SET);

if (dataIn!=NULL)
  return dataIn;
return NULL;
}


void setStandby(){
  uint8_t data[] = {0x80,0x00};
  SPItransferCmd(SPIWrite,data,NULL,2);
}
void setBufferbase(){
  uint8_t data[] = {0x8F,0x00,0x00};
  SPItransferCmd(SPIWrite,data,NULL,3);
}
void setPackettype(){
  uint8_t data[] = {0x8A,0x01};
  SPItransferCmd(SPIWrite,data,NULL,2);
}
void setFallback(){
  uint8_t data[] = {0x93,0x20};
  SPItransferCmd(SPIWrite,data,NULL,2);
}
void setCADParam(){
  uint8_t data[] = {0x88,0x03,0x16,0x0A,0x00,0x00,0x00,0x00};
  SPItransferCmd(SPIWrite,data,NULL,8);
}
void clrIRQ(){
  uint8_t data[] = {0x02,0x03,0xFF};
  SPItransferCmd(SPIWrite,data,NULL,3);
}
void setDIO(){
  uint8_t data[] = {0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  SPItransferCmd(SPIWrite,data,NULL,9);
}
void setDIORead(){
  uint8_t data[] = {0x08,0x02,0x62,0x00,0x02,0x00,0x00,0x00,0x00};
  SPItransferCmd(SPIWrite,data,NULL,9);
}
void setDIOTransmit(){
  uint8_t data[] = {0x08,0x02,0x01,0x00,0x01,0x00,0x00,0x00,0x00};
  SPItransferCmd(SPIWrite,data,NULL,9);
}
void calibrate(){
  uint8_t data[] = {0x89,0x7F};
  SPItransferCmd(SPIWrite,data,NULL,2);
}
void getDevErrors(uint8_t *returnData){
  uint8_t data[] = {0x17,0x00,0x00,0x00};
  SPItransferCmd(SPIRead,data,returnData,4);
}

void setDIO3(){
  uint8_t data[] = {0x97,0x00,0x00,0x01,0x40};
  SPItransferCmd(SPIWrite,data,NULL,5);
}

void setmodParam(){
  uint8_t data[] = {0x8b,0x09,0x04,0x03,0x00};
  SPItransferCmd(SPIWrite,data,NULL,5);
}

void setSyncword(){
  uint8_t data[] = {0x0D,0x07,0x40,0x14,0x24};
  SPItransferCmd(SPIWrite,data,NULL,5);
}

void setReadIQpol(uint8_t *returnData){
  uint8_t data[] = {0x1d,0x07,0x36,0x00,0x00};
  SPItransferCmd(SPIRead,data,returnData,5);
}


void fixIQ(){
  uint8_t data[] = {0x0D,0x07,0x36,0x09};
  SPItransferCmd(SPIWrite,data,NULL,4);
}

void setPacketparam(){
  uint8_t data[] = {0x8c,0x00,0x08,0x00,0xFF,0x01,0x00};
  SPItransferCmd(SPIWrite,data,NULL,7);
}


void SetOCP(){
  uint8_t data[] = {0x0D,0x08,0xE7,0x18};
  SPItransferCmd(SPIWrite,data,NULL,4);
}

void setDIO2(){
  uint8_t data[] = {0x9D,0x01};
  SPItransferCmd(SPIWrite,data,NULL,2);
}
void regParam(){
  uint8_t data[] = {0x96,0x01};
  SPItransferCmd(SPIWrite,data,NULL,2);
}


void calibrateImage(){
  uint8_t data[] = {0x98,0x6B,0x6F};
  SPItransferCmd(SPIWrite,data,NULL,3);
}

void setRFfreq(){
  uint8_t data[] = {0x86,0x1B,0x20,0x00,0x00};
  SPItransferCmd(SPIWrite,data,NULL,5);
}


void getOCP(){
  uint8_t data[] = {0x1D,0x08,0xE7,0x00,0x00};
  SPItransferCmd(SPIRead,data,NULL,5);
}

void SetPaconfig(){
  uint8_t data[] = {0x95,0x04,0x07,0x00,0x01};
  SPItransferCmd(SPIWrite,data,NULL,5);
}

void SetTxparameters(){
  uint8_t data[] = {0x8e,0x0A,0x04};
  SPItransferCmd(SPIWrite,data,NULL,3);
}
void getPacket(){
  uint8_t packettype[2];
  uint8_t data[] = {0x11,0x00,0x00};
  SPItransferCmd(SPIRead,data,packettype,3);
  /*print("Packet Type:\t\t");
  if (packettype[1]==(uint8_t)1)
    println("LoRa");
  else
    println("Other");
	*/
}

void getBufferstat(uint8_t* bffr){
  uint8_t data[] = {0x13,0x00,0x00};
  SPItransferCmd(SPIRead,data,bffr,3);
  bffr[1]-=1;
}

void readBuffer(uint8_t *recMessage,uint8_t len,uint8_t start){
  waitBusy();
  HAL_GPIO_WritePin( GPIOB, CS_RADIO_Pin, GPIO_PIN_RESET);
  /*SPI.transfer(0x1E);
  SPI.transfer(start);
  SPI.transfer(0x00);*/
  uint8_t data[]={0x1e,start,0x00};
  HAL_SPI_Transmit(&hspi1, data, 3,100);
  HAL_SPI_Receive(&hspi1, recMessage, len, 100);
  /*for(int i =0;i<len;i++)
    recMessage[i]=SPI.transfer(0x00);*/
  HAL_GPIO_WritePin( GPIOB, CS_RADIO_Pin, GPIO_PIN_RESET);
}

void writePayload(char* payload){
  uint8_t data[sizeof payload +2];
  data[0]=0x0E;
  data[1]=0x00;
  for(int i =0; i<sizeof payload;i++)
    data[i+2] = payload[i];
  SPItransferCmd(SPIWrite,data,NULL,3);
}



void Radiobegin(void){
  //SetStandby
  setStandby();
  //Setbufferbase
  setBufferbase();
  //Set packet type
  setPackettype();
  //Fallback mode
  setFallback();
  //Set CADParam
  setCADParam();
  //ClearIRQ
  clrIRQ();
  //SetDIO
  setDIO();
  //calibrate
  calibrate();
  //set standby
  setStandby();
  //SetDIO3
  setDIO3();
  //SetModPara
  setmodParam();
  //SetSyncword
  setSyncword();
  //SetPKTPara
  setPacketparam();
  //SetOCP
  SetOCP();
  //SetDIO2
  setDIO2();
  //RegulatorParam
  regParam();
  //Calibrate Image
  calibrateImage();
  //SetRFfreq
  setRFfreq();
  //SetPaconfig
  SetPaconfig();
  //SetTxparameters
  SetTxparameters();
}

#endif /* INC_RADIOCOMMANDS_H_ */
