/*
 * SX1262.h
 *
 *  Created on: Nov 12, 2021
 *      Author: Alex (modified by Josh)
 *
 Usage
 radio_on() - activates the radio and sets everything up
 radio_receive(hspi, timeout) - attempt to receive a LoRa packet into recMessage, for up to timeout (ms)
 radio_transmit(hspi, timeout, message) - attempts to send message as LoRa packet, for up to timeout (ms)
 radio_off() - low power mode, call to radio_on() should be made to exit
 eg.
    radio_on(hspi1);
    if (radio_receive(hspi1, 30000))
	{
		println("Packet received...");
		print_lora_pkt();
		printint32_t(getPacketRSSI(hspi1));
		println("dBm");
		printint32_t(getSignalRSSI(hspi1));
		println("dBm");
		printint32_t(getSNR(hspi1));
		println("dB\n");
	}
	else
		println("No packet received...\n");*/

	/*println("Attempting transmission...");
	const char* msg = "WM";
	if (radio_transmit(hspi1, 30000, msg))
		println("Transmission successful");
	else
		println("Transmission failed.");

 */

#ifndef INC_SX1262_H_
#define INC_SX1262_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h" 	// gives access to pin defines and HAL SPI

#define STANDBY_MODE 0
#define RECEIVE_MODE 1
#define TRANSMIT_MODE 2
#define SLEEP_MODE 3

#define SPIRead 0
#define SPIWrite 1

#define STANDBY_CMD 0x80
#define SLEEP_CMD 0x84
#define SETBUFFERBASEADDRESS_CMD 0x8F
#define SETPACKETTYPE_CMD 0x8A
#define SETTXFALLBACK_CMD 0x93
#define SETCADPARAMS_CMD 0x88
#define CLRIRQ_CMD 0x02
#define SETDIOPARAMS_CMD 0x08
#define CALIBRATE_CMD 0x89
#define SETDIO3TCXO_CMD 0x97
#define SETMODPARAMS_CMD 0x8B
#define WRITEREG_CMD 0x0D
#define READREG_CMD 0x1D
#define SETPACKETPARAMS_CMD 0x8C
#define SETDIO2RFSW_CMD 0x9D
#define SETREGMODE_CMD 0x96
#define CALIB_IMG_CMD 0x98
#define SETRFFREQ_CMD 0x86
#define SETPACONFIG_CMD 0x95
#define SETTXPARAMS_CMD 0x8E
#define SETRX_CMD 0X82
#define GETRXSTAT_CMD 0x13
#define GETPKTSTAT_CMD 0x14
#define READBUF_CMD 0x1E
#define WRITEBUF_CMD 0x0E
#define SETTX_CMD 0x83
#define GETPKTTYPE_CMD 0x11
#define GETIRQ_CMD 0x12
#define GETSTATUS_CMD 0xC0
#define GETERRORS_CMD 0x17
#define CLRERRORS_CMD 0x07

#define STANDBY_RC 0x00
#define STANDBY_XOSC 0x01
#define LORA_PKT 0x01
#define STDBY_RC_MODE 0x20
#define STDBY_XOSC_MODE 0x30
#define CALIBRATE_ALL 0x7F
#define LORA_SF_9 0x09
#define LORA_SF_12 0x0C
#define LORA_BW_10k 0x08
#define LORA_BW_125k 0x04
#define LORA_FEC_CR47 0x03
#define LORA_FEC_CR48 0x04
#define LORA_LDRO_OFF 0x00
#define LORA_LDRO_ON 0x01
#define LORA_VAR_HEADER 0x00
#define LORA_MAX_PAYLOAD 0xFF
#define LORA_60mA 0x18
#define LORA_140mA 0x38
#define VERBOSE 0x01

char recMessage[LORA_MAX_PAYLOAD + 3];
uint8_t rxpacketInfo[4];
uint8_t radioMode;

// Ensures all the GPIO and SPI is configured accordingly
void radio_HAL_setup();
uint8_t* radioSPItransfer(SPI_HandleTypeDef hspi, uint8_t *dataOut, uint8_t *dataIn, uint16_t numBytes);
void radio_reset(void);
void setStandby(SPI_HandleTypeDef hspi);
void setSleep(SPI_HandleTypeDef hspi);
void setBufferbase(SPI_HandleTypeDef hspi);
void setPacketType(SPI_HandleTypeDef hspi);
void setFallback(SPI_HandleTypeDef hspi);
void setCADParam(SPI_HandleTypeDef hspi);
void clrIRQ(SPI_HandleTypeDef hspi);
void clrErrors(SPI_HandleTypeDef hspi);
void setDIO(SPI_HandleTypeDef hspi);
void setDIORead(SPI_HandleTypeDef hspi);
void setDIOTransmit(SPI_HandleTypeDef hspi);
void calibrate(SPI_HandleTypeDef hspi);
void setDIO3(SPI_HandleTypeDef hspi);
void setmodParam(SPI_HandleTypeDef hspi, uint8_t lora_sf, uint8_t lora_bw, uint8_t lora_cr, uint8_t lora_ldro);
void setSyncword(SPI_HandleTypeDef hspi);
void getSyncword(SPI_HandleTypeDef hspi);
void getWhitening(SPI_HandleTypeDef hspi);
void getCRC(SPI_HandleTypeDef hspi);
void getRxGain(SPI_HandleTypeDef hspi);
void setRxGainBoosted(SPI_HandleTypeDef hspi);
void getOCP(SPI_HandleTypeDef hspi);
void getXTTrim(SPI_HandleTypeDef hspi);
void getIRQStatus(SPI_HandleTypeDef hspi);
void getStatus(SPI_HandleTypeDef hspi);
uint8_t getErrors(SPI_HandleTypeDef hspi);
uint8_t getErrorsSilent(SPI_HandleTypeDef hspi);
void setPacketParam(SPI_HandleTypeDef hspi, uint8_t lora_header, uint8_t lora_packet_length);
void setOCP(SPI_HandleTypeDef hspi, uint8_t lora_current);
void setDIO2(SPI_HandleTypeDef hspi);
void regParam(SPI_HandleTypeDef hspi);
void calibrateImage(SPI_HandleTypeDef hspi);
void setRFfreq(SPI_HandleTypeDef hspi, uint32_t freq);
void setPaconfig(SPI_HandleTypeDef hspi);
void setTxparameters(SPI_HandleTypeDef hspi);
uint8_t checkPacketLoRa(SPI_HandleTypeDef hspi);
void getRxBufferStatus(SPI_HandleTypeDef hspi, uint8_t *buffer);
int32_t getPacketRSSI(SPI_HandleTypeDef hspi);
int32_t getSignalRSSI(SPI_HandleTypeDef hspi);
int32_t getSNR(SPI_HandleTypeDef hspi);
void readBuffer(SPI_HandleTypeDef hspi, char *recMessage, uint8_t message_len, uint8_t offset);
void writePayload(SPI_HandleTypeDef hspi, const char *payload, uint16_t msglen);
void radio_begin(SPI_HandleTypeDef hspi);
void radio_sitrep(SPI_HandleTypeDef hspi);
void radio_on(SPI_HandleTypeDef hspi);
void radio_off(SPI_HandleTypeDef hspi);
uint8_t radio_receive(SPI_HandleTypeDef hspi, uint32_t timeout_ms);
uint8_t setRX(SPI_HandleTypeDef hspi, uint32_t timeout_ms);
uint8_t radio_transmit(SPI_HandleTypeDef hspi, uint32_t timeout_ms, const char *message);
uint8_t setTX(SPI_HandleTypeDef hspi, uint32_t timeout_ms, const char *message);
uint8_t transmit_demo(SPI_HandleTypeDef hspi);

#ifdef __cplusplus
}
#endif

#endif /* INC_SX1262_H_ */
