/*
 * SX1262.c
 *
 *  Created on: Nov 12, 2021
 *      Author: Alex (modified by Josh)
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "SX1262.h"

// Ensures all the GPIO and SPI is configured accordingly
void radio_HAL_setup()
{
	radioMode = SLEEP_MODE;

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// Ensures Radio Chip Select Pin is set as an output (HIGH)
	GPIO_InitStruct.Pin = CS_RADIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(CS_RADIO_GPIO_Port, &GPIO_InitStruct);
	HAL_GPIO_WritePin(CS_RADIO_GPIO_Port, CS_RADIO_Pin, GPIO_PIN_SET);

	// Ensures Radio !Reset Pin is set as an output (HIGH)
	GPIO_InitStruct.Pin = RADIO_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(RADIO_RST_GPIO_Port, &GPIO_InitStruct);
	HAL_GPIO_WritePin(RADIO_RST_GPIO_Port, RADIO_RST_Pin, GPIO_PIN_SET);

	// Ensures Radio Busy Pin is set as an input
	GPIO_InitStruct.Pin = RADIO_BUSY_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT; // digital Input
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(RADIO_BUSY_GPIO_Port, &GPIO_InitStruct);

	// Ensures Radio DIO1 Pin is set as an input (can be set up as an interrupt, no need though)
	GPIO_InitStruct.Pin = RADIO_DIO1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(RADIO_DIO1_GPIO_Port, &GPIO_InitStruct);

	// Allows access to the CPU cycle counter, useful for timing purposes
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// Uses HAL SPI handle to transfer data between the microcontroller and the radio
uint8_t* radioSPItransfer(SPI_HandleTypeDef hspi, uint8_t *dataOut, uint8_t *dataIn, uint16_t numBytes)
{
	// wait for radio to not be busy, or just timeout
	uint8_t busy = HAL_GPIO_ReadPin(RADIO_BUSY_GPIO_Port, RADIO_BUSY_Pin);
	if (busy == 1)
	{
		uint32_t start_clks = DWT->CYCCNT; //start timer
		while ((busy == 1))
		{
			busy = HAL_GPIO_ReadPin(RADIO_BUSY_GPIO_Port, RADIO_BUSY_Pin);
			uint32_t ms_elapsed = (DWT->CYCCNT - start_clks)/(HAL_RCC_GetSysClockFreq()/1000);
			if (ms_elapsed > 100)
				return NULL;
		}
	}

	HAL_GPIO_WritePin(CS_RADIO_GPIO_Port, CS_RADIO_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi, dataOut, dataIn, numBytes, 100);
	HAL_GPIO_WritePin(CS_RADIO_GPIO_Port, CS_RADIO_Pin, GPIO_PIN_SET);

	if (dataIn != NULL)
	{
		return dataIn;
	}
	return NULL;
}

//Quick function to reset the radio back to factory settings
void radio_reset(void)
{
	HAL_GPIO_WritePin(RADIO_RST_GPIO_Port, RADIO_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(RADIO_RST_GPIO_Port, RADIO_RST_Pin, GPIO_PIN_SET);
}

void setStandby(SPI_HandleTypeDef hspi)
{
	uint8_t data[] = {STANDBY_CMD, STANDBY_RC};
	uint8_t rx_data[2] = {0};
	radioSPItransfer(hspi, data, rx_data, 2);
}

void setSleep(SPI_HandleTypeDef hspi)
{
	uint8_t data[] = {SLEEP_CMD, 0x00};
	uint8_t rx_data[2] = {0};
	radioSPItransfer(hspi, data, rx_data, 2);
	HAL_Delay(1);
}

void setBufferbase(SPI_HandleTypeDef hspi)
{
	uint8_t data[] = {SETBUFFERBASEADDRESS_CMD, 0x00, 0x00};
	uint8_t rx_data[3] = {0};
	radioSPItransfer(hspi, data, rx_data, 3);
}

void setPacketType(SPI_HandleTypeDef hspi)
{
	uint8_t data[] = {SETPACKETTYPE_CMD, LORA_PKT};
	uint8_t rx_data[2] = {0};
	radioSPItransfer(hspi, data, rx_data, 2);
}

void setFallback(SPI_HandleTypeDef hspi)
{
	uint8_t data[] = {SETTXFALLBACK_CMD, STDBY_RC_MODE};
	uint8_t rx_data[2] = {0};
	radioSPItransfer(hspi, data, rx_data, 2);
}

void setCADParam(SPI_HandleTypeDef hspi)
{
	uint8_t data[] = {SETCADPARAMS_CMD, 0x03, 0x16, 0x0A, 0x00, 0x00, 0x00, 0x00};
	uint8_t rx_data[8] = {0};
	radioSPItransfer(hspi, data, rx_data, 8);
}

void clrIRQ(SPI_HandleTypeDef hspi)
{
	uint8_t data[] = {CLRIRQ_CMD, 0x03, 0xFF};
	uint8_t rx_data[3] = {0};
	radioSPItransfer(hspi, data, rx_data, 3);
}

void clrErrors(SPI_HandleTypeDef hspi)
{
	uint8_t data[] = {CLRERRORS_CMD, 0x00};
	uint8_t rx_data[2] = {0};
	radioSPItransfer(hspi, data, rx_data, 2);
}

void setDIO(SPI_HandleTypeDef hspi)
{
	uint8_t data[] = {SETDIOPARAMS_CMD, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t rx_data[9] = {0};
	radioSPItransfer(hspi, data, rx_data, 9);
}

void setDIORead(SPI_HandleTypeDef hspi)
{
	uint8_t data[] = {SETDIOPARAMS_CMD, 0x02, 0x62, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00};
	uint8_t rx_data[9] = {0};
	radioSPItransfer(hspi, data, rx_data, 9);
}

void setDIOTransmit(SPI_HandleTypeDef hspi)
{
	uint8_t data[] = {SETDIOPARAMS_CMD, 0x02, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
	uint8_t rx_data[9] = {0};
	radioSPItransfer(hspi, data, rx_data, 9);
}

void calibrate(SPI_HandleTypeDef hspi)
{
	uint8_t data[] = {CALIBRATE_CMD, CALIBRATE_ALL};
	uint8_t rx_data[2] = {0};
	radioSPItransfer(hspi, data, rx_data, 2);
	HAL_Delay(5);
}

void setDIO3(SPI_HandleTypeDef hspi)
{
	uint8_t data[] = {SETDIO3TCXO_CMD, 0x00, 0x00,  0x01, 0x40}; //1.8V and 5ms
	uint8_t rx_data[5] = {0};
	radioSPItransfer(hspi, data, rx_data, 5);
	HAL_Delay(5);
}

void setmodParam(SPI_HandleTypeDef hspi, uint8_t lora_sf, uint8_t lora_bw, uint8_t lora_cr, uint8_t lora_ldro)
{
	uint8_t data[] = {SETMODPARAMS_CMD, lora_sf, lora_bw, lora_cr, lora_ldro};
	uint8_t rx_data[5] = {0};
	radioSPItransfer(hspi, data, rx_data, 5);
}


void setSyncword(SPI_HandleTypeDef hspi)
{
	uint8_t data[] = {WRITEREG_CMD, 0x07, 0x40, 0x14, 0x24};
	uint8_t rx_data[5] = {0};
	radioSPItransfer(hspi, data, rx_data, 5);
}

void getSyncword(SPI_HandleTypeDef hspi)
{
	uint8_t data[] = {READREG_CMD, 0x07, 0x40, 0x00, 0x00, 0x00};
    uint8_t syncword[6]= {0};
    radioSPItransfer(hspi ,data, syncword, 6);
    print("Syncword:\t\t");
    printuint8_t(syncword[4]);
    print(" ");
    printuint8_t(syncword[5]);
    println("");
}

void getWhitening(SPI_HandleTypeDef hspi)
{
	uint8_t data[] = {READREG_CMD, 0x06, 0xB8, 0x00, 0x00, 0x00};
    uint8_t whitening[6]= {0};
    radioSPItransfer(hspi ,data, whitening, 6);
    print("Whitening:\t\t");
    printuint8_t(whitening[4]);
    print(" ");
    printuint8_t(whitening[5]);
    println("");
}


void getCRC(SPI_HandleTypeDef hspi)
{
	uint8_t data[] = {READREG_CMD, 0x06, 0xBC, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t crc[8]= {0};
    radioSPItransfer(hspi ,data, crc, 8);
    print("CRC:\t\t\t");
    printuint8_t(crc[4]);
    print(" ");
    printuint8_t(crc[5]);
    print(" ");
    printuint8_t(crc[6]);
    print(" ");
    printuint8_t(crc[7]);
    println("");
}

void getRxGain(SPI_HandleTypeDef hspi)
{
	uint8_t data[] = {READREG_CMD, 0x08, 0xAC, 0x00};
    uint8_t rxgain[4]= {0};
    radioSPItransfer(hspi ,data, rxgain, 4);
    print("Rx Gain:\t\t");
    printuint8_t(rxgain[4]);
    println("");
}

void setRxGainBoosted(SPI_HandleTypeDef hspi)
{
	uint8_t data[] = {WRITEREG_CMD, 0x08, 0xAC, 0x96};
    uint8_t rxgain[4]= {0};
    radioSPItransfer(hspi ,data, rxgain, 4);
}

void getOCP(SPI_HandleTypeDef hspi)
{
	uint8_t data[] = {READREG_CMD, 0x08, 0xE7, 0x00, 0x00};
    uint8_t ocp[5]= {0};
    radioSPItransfer(hspi ,data, ocp, 5);
    print("OCP:\t\t\t");
    printuint8_t(ocp[4]);
    println("");
}

void getXTTrim(SPI_HandleTypeDef hspi)
{
	uint8_t data[] = {READREG_CMD, 0x09, 0x11, 0x00, 0x00, 0x00};
    uint8_t xttrim[6]= {0};
    radioSPItransfer(hspi ,data, xttrim, 6);
    print("XT Trim:\t\t");
    printuint8_t(xttrim[4]);
    print(" ");
    printuint8_t(xttrim[5]);
    println("");
}

void getIRQStatus(SPI_HandleTypeDef hspi)
{
	uint8_t data[] = {GETIRQ_CMD, 0x00, 0x00, 0x00};
	uint8_t irq_stat[4] = {0};
	radioSPItransfer(hspi, data, irq_stat, 4);
	print("IRQ:\t\t\t");
	printuint8_t(irq_stat[2]);
	print(" ");
	printuint8_t(irq_stat[3]);
	println("");
}

void getStatus(SPI_HandleTypeDef hspi)
{
	uint8_t data[] = {GETSTATUS_CMD, 0x00};
	uint8_t status[2] = {0};
	radioSPItransfer(hspi, data, status, 2);
	print("STATUS:\t\t\t");
	printuint8_t(status[1]);
	println("");
}

uint8_t getErrors(SPI_HandleTypeDef hspi)
{
	uint8_t data[] = {GETERRORS_CMD, 0x00, 0x00, 0x00};
	uint8_t errors[4] = {0};
	radioSPItransfer(hspi, data, errors, 4);
	print("ERRORS:\t\t\t");
	printuint8_t(errors[2]);
	print(" ");
	printuint8_t(errors[3]);
	println("");
	return errors[3];
}

uint8_t getErrorsSilent(SPI_HandleTypeDef hspi)
{
	uint8_t data[] = {GETERRORS_CMD, 0x00, 0x00, 0x00};
	uint8_t errors[4] = {0};
	radioSPItransfer(hspi, data, errors, 4);
	return errors[3];
}

void setPacketParam(SPI_HandleTypeDef hspi, uint8_t lora_header, uint8_t lora_packet_length)
{
	uint8_t data[] = {SETPACKETPARAMS_CMD, 0x00, 0x0C, lora_header, lora_packet_length, 0x01, 0x00};
	uint8_t rx_data[7] = {0};
	radioSPItransfer(hspi, data, rx_data, 7);
}

void setOCP(SPI_HandleTypeDef hspi, uint8_t lora_current)
{
	uint8_t data[] = {WRITEREG_CMD, 0x08, 0xE7, lora_current};
	uint8_t rx_data[4] = {0};
	radioSPItransfer(hspi, data, rx_data, 4);
}

void setDIO2(SPI_HandleTypeDef hspi)
{
	uint8_t data[] = {SETDIO2RFSW_CMD, 0x01};
	uint8_t rx_data[2] = {0};
	radioSPItransfer(hspi, data, rx_data, 2);
}

void regParam(SPI_HandleTypeDef hspi)
{
	// DC to DC Converter, and LDO used
	uint8_t data[] = {SETREGMODE_CMD, 0x01};
	uint8_t rx_data[2] = {0};
	radioSPItransfer(hspi, data, rx_data, 2);
}

void calibrateImage(SPI_HandleTypeDef hspi)
{
	// Calibrate for 430-440MHz
	uint8_t data[] = {CALIB_IMG_CMD, 0x6B, 0x6F};
	uint8_t rx_data[3] = {0};
	radioSPItransfer(hspi, data, rx_data, 3);
}

// Accepts frequency in Hz, use correct calibrateImage first
void setRFfreq(SPI_HandleTypeDef hspi, uint32_t freq)
{
	//convert int32 into correct bytes
	int32_t corr_freq = freq / 0.95367431640625;
	uint8_t data[5];
	data[0] = SETRFFREQ_CMD;
	data[1] = (uint8_t)((corr_freq >> 24) & 0xFF);
	data[2] = (uint8_t)((corr_freq >> 16) & 0xFF);
	data[3] = (uint8_t)((corr_freq >>  8) & 0xFF);
	data[4] = (uint8_t)((corr_freq      ) & 0xFF);
	uint8_t rx_data[5] = {0};
	radioSPItransfer(hspi, data, rx_data, 5);
}

// Think very carefully before changing these values!
void setPaconfig(SPI_HandleTypeDef hspi)
{
	uint8_t data[] = {SETPACONFIG_CMD, 0x04, 0x07, 0x00, 0x01};	//+22dBm!
	uint8_t rx_data[5] = {0};
	radioSPItransfer(hspi, data, rx_data, 5);
}

// Think very carefully before changing these values!
void setTxparameters(SPI_HandleTypeDef hspi)
{
	uint8_t data[] = {SETTXPARAMS_CMD, 0x16, 0x07};	//+22dBm!
	uint8_t rx_data[3] = {0};
	radioSPItransfer(hspi, data, rx_data, 3);
}

uint8_t checkPacketLoRa(SPI_HandleTypeDef hspi)
{
	uint8_t data[] = {GETPKTTYPE_CMD,0x00,0x00};
    uint8_t packettype[3]= {0};
    radioSPItransfer(hspi, data, packettype,3);
    if (packettype[1]==(uint8_t)1)
		return 1;
	return 0;
}

void getRxBufferStatus(SPI_HandleTypeDef hspi, uint8_t *buffer)
{
	uint8_t data[] = {GETRXSTAT_CMD, 0x00, 0x00, 0x00};
	radioSPItransfer(hspi, data, buffer, 4);
	buffer[2] -= 1;
}


int32_t getPacketRSSI(SPI_HandleTypeDef hspi)
{
	uint8_t data[] = {GETPKTSTAT_CMD, 0x00, 0x00, 0x00, 0x00};
	uint8_t pktstat[5] = {0};
	radioSPItransfer(hspi, data, pktstat, 5);
	int32_t pktrssi = (-1.0  * (pktstat[2] / 2.0));
	return pktrssi;
}

int32_t getSignalRSSI(SPI_HandleTypeDef hspi)
{
	uint8_t data[] = {GETPKTSTAT_CMD, 0x00, 0x00, 0x00, 0x00};
	uint8_t pktstat[5] = {0};
	radioSPItransfer(hspi, data, pktstat, 5);
	int32_t sigrssi = (-1.0  * (pktstat[4] / 2.0));
	return sigrssi;
}

int32_t getSNR(SPI_HandleTypeDef hspi)
{
	uint8_t data[] = {GETPKTSTAT_CMD, 0x00, 0x00, 0x00, 0x00};
	uint8_t pktstat[5] = {0};
	radioSPItransfer(hspi, data, pktstat, 5);
	int32_t snr = (4.0  * pktstat[3]);
	return snr;
}

void readBuffer(SPI_HandleTypeDef hspi, char *recMessage, uint8_t message_len, uint8_t offset)
{
	for (uint16_t i = 0; i < (sizeof recMessage); i++)
		recMessage[i] = 0;

	uint8_t data[LORA_MAX_PAYLOAD + 3] = {0};	//yes this is wasteful, but much tidier
	data[0] = READBUF_CMD;
	data[1] = offset;
	radioSPItransfer(hspi, data, (uint8_t *)recMessage, message_len + 3);
}

void writePayload(SPI_HandleTypeDef hspi, const char *payload, uint16_t msglen)
{
	uint8_t data[LORA_MAX_PAYLOAD + 2] = {0};
	data[0] = WRITEBUF_CMD;
	for(uint16_t i  = 0; i < msglen; i++)
	{
		data[i+2] = payload[i];
	}
	uint8_t rx_data[LORA_MAX_PAYLOAD + 2] = {0};
	radioSPItransfer(hspi, data, rx_data, msglen + 2);
}

// Goes through all the startup commands to get the radio into LoRa standby mode
// Call this after a reset
void radio_begin(SPI_HandleTypeDef hspi)
{
	setStandby(hspi);
	setBufferbase(hspi);
	setPacketType(hspi);
	setFallback(hspi);
	setCADParam(hspi);
	clrIRQ(hspi);
	setDIO(hspi);
	calibrate(hspi);
	setStandby(hspi);
	setDIO3(hspi);
	setmodParam(hspi, LORA_SF_12, LORA_BW_10k, LORA_FEC_CR48, LORA_LDRO_ON);
	setSyncword(hspi);
	setPacketParam(hspi, LORA_VAR_HEADER, LORA_MAX_PAYLOAD);
	setOCP(hspi, LORA_60mA);
	setDIO2(hspi);
	regParam(hspi);
	calibrateImage(hspi);
	setRFfreq(hspi, 434000000);
	setPaconfig(hspi);
	setTxparameters(hspi);
}

void radio_sitrep(SPI_HandleTypeDef hspi)
{
	println("~~~~~~~~~~  RADIO SITREP  ~~~~~~~~");
	getSyncword(hspi);
	getWhitening(hspi);
	getCRC(hspi);
	getRxGain(hspi);
	getOCP(hspi);
	getXTTrim(hspi);
	getIRQStatus(hspi);
	getStatus(hspi);
	getErrors(hspi);
	println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
}

// Configures the STM32 and SX1262 for LoRa radio
// Call this on startup, or to wake radio from sleep/off mode
void radio_on(SPI_HandleTypeDef hspi)
{
	radio_HAL_setup();
	radio_reset();
	radio_begin(hspi);
	radioMode = STANDBY_MODE;
	//println("Radio activated.");
}

// Puts the radio into a sleep mode for low current
// radio_on() should be called after to bring the radio back into operation
void radio_off(SPI_HandleTypeDef hspi)
{
	setStandby(hspi);
	setSleep(hspi);
	radioMode = SLEEP_MODE;
	//println("Radio deactivated.");
}

uint8_t radio_receive(SPI_HandleTypeDef hspi, uint32_t timeout_ms)
{
	//println("Attempting to receive a LoRa packet...");
	if (radioMode == SLEEP_MODE)
		radio_on(hspi);

	radioMode = RECEIVE_MODE;
	setStandby(hspi);
	setDIORead(hspi);
	setBufferbase(hspi);
	clrIRQ(hspi);
	setRxGainBoosted(hspi);
	setPacketParam(hspi, LORA_VAR_HEADER, LORA_MAX_PAYLOAD);
	return setRX(hspi, timeout_ms);
}

uint8_t setRX(SPI_HandleTypeDef hspi, uint32_t timeout_ms)
{
	uint8_t data[] = {SETRX_CMD, 0xFF, 0XFF, 0XFF};
	uint8_t rx_data[4] = {0};
	radioSPItransfer(hspi, data, rx_data, 4);
	// wait for radio to receive something, or just timeout
	uint32_t start_clks = DWT->CYCCNT; //start timer
	uint32_t ms_elapsed = 0;
	while ((HAL_GPIO_ReadPin(RADIO_DIO1_GPIO_Port, RADIO_DIO1_Pin) == 0) && (ms_elapsed < timeout_ms))
	{
		ms_elapsed = (DWT->CYCCNT - start_clks)/(HAL_RCC_GetSysClockFreq()/1000);
	}
	radioMode = STANDBY_MODE;
	setStandby(hspi);
	if (HAL_GPIO_ReadPin(RADIO_DIO1_GPIO_Port, RADIO_DIO1_Pin) == 1)
	{
		uint8_t buffer_stat[4];
		getRxBufferStatus(hspi, buffer_stat);
		readBuffer(hspi, recMessage, buffer_stat[2], buffer_stat[3]);
		return 1;
	}
	return 0;
}

uint8_t radio_transmit(SPI_HandleTypeDef hspi, uint32_t timeout_ms, const char *message)
{
	//println("Attempting to transmit a LoRa packet");
	if (radioMode == SLEEP_MODE)
		radio_on(hspi);

	radioMode = TRANSMIT_MODE;
	setStandby(hspi);
	uint16_t msglen = sizeof message;
	if (msglen > LORA_MAX_PAYLOAD)
		return 0;

	setPacketParam(hspi, LORA_VAR_HEADER, msglen);
	setDIOTransmit(hspi);
	setBufferbase(hspi);
	clrIRQ(hspi);
	writePayload(hspi, message, msglen);
	return setTX(hspi, timeout_ms, message);
}


uint8_t setTX(SPI_HandleTypeDef hspi, uint32_t timeout_ms, const char *message)
{
	if (timeout_ms > 20000)
		timeout_ms = 20000;	// Place upper limit on timeout of 20 seconds

	uint8_t data[] = {SETTX_CMD, 0x00, 0X0, 0X00};
	uint8_t rx_data[4] = {0};
	radioSPItransfer(hspi, data, rx_data, 4);

	// wait for radio to finish transmitting, or just timeout
	uint32_t start_clks = DWT->CYCCNT; //start timer
	uint32_t ms_elapsed = 0;
	while ((HAL_GPIO_ReadPin(RADIO_DIO1_GPIO_Port, RADIO_DIO1_Pin) == 0) && (ms_elapsed < timeout_ms))
	{
		ms_elapsed = (DWT->CYCCNT - start_clks)/(HAL_RCC_GetSysClockFreq()/1000);
	}
	radioMode = STANDBY_MODE;
	setStandby(hspi);
	if (HAL_GPIO_ReadPin(RADIO_DIO1_GPIO_Port, RADIO_DIO1_Pin) == 1)
		return 1;
	return 0;
}

#ifdef __cplusplus
}
#endif
