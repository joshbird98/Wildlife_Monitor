/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
extern	SPI_HandleTypeDef hspi1;
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void print(char _out[]);
void println(char _out[]);
void printuint32_t(uint32_t value);
void printuint16_t(uint16_t value);
void printint16_t(int16_t value);
void printuint8_t(uint8_t value);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define AUDIO_SAMPLE_RATE 44100
#define CHRG_PG_Pin GPIO_PIN_2
#define CHRG_PG_GPIO_Port GPIOE
#define CHRG_STAT2_Pin GPIO_PIN_3
#define CHRG_STAT2_GPIO_Port GPIOE
#define CHRG_STAT1_Pin GPIO_PIN_4
#define CHRG_STAT1_GPIO_Port GPIOE
#define CS_EINK_Pin GPIO_PIN_7
#define CS_EINK_GPIO_Port GPIOF
#define DC_EINK_Pin GPIO_PIN_8
#define DC_EINK_GPIO_Port GPIOF
#define RST_EINK_Pin GPIO_PIN_9
#define RST_EINK_GPIO_Port GPIOF
#define BUSY_EINK_Pin GPIO_PIN_10
#define BUSY_EINK_GPIO_Port GPIOF
#define RADIO_TX_Pin GPIO_PIN_0
#define RADIO_TX_GPIO_Port GPIOH
#define RADIO_RX_Pin GPIO_PIN_1
#define RADIO_RX_GPIO_Port GPIOH
#define AN_VUREG_Pin GPIO_PIN_0
#define AN_VUREG_GPIO_Port GPIOC
#define AN_BAT_Pin GPIO_PIN_1
#define AN_BAT_GPIO_Port GPIOC
#define AN_INTMIC_Pin GPIO_PIN_2
#define AN_INTMIC_GPIO_Port GPIOC
#define MODE_Pin GPIO_PIN_3
#define MODE_GPIO_Port GPIOC
#define CS_MIC_GAIN_Pin GPIO_PIN_2
#define CS_MIC_GAIN_GPIO_Port GPIOA
#define CS_MIC_ADC_Pin GPIO_PIN_3
#define CS_MIC_ADC_GPIO_Port GPIOA
#define RADIO_DIO1_Pin GPIO_PIN_4
#define RADIO_DIO1_GPIO_Port GPIOC
#define RADIO_BUSY_Pin GPIO_PIN_5
#define RADIO_BUSY_GPIO_Port GPIOC
#define RADIO_RST_Pin GPIO_PIN_0
#define RADIO_RST_GPIO_Port GPIOB
#define CS_RADIO_Pin GPIO_PIN_1
#define CS_RADIO_GPIO_Port GPIOB
#define MIC_ACTIVE_Pin GPIO_PIN_2
#define MIC_ACTIVE_GPIO_Port GPIOB
#define CARD_Pin GPIO_PIN_7
#define CARD_GPIO_Port GPIOC
#define BLUE_3_Pin GPIO_PIN_3
#define BLUE_3_GPIO_Port GPIOD
#define RED_3_Pin GPIO_PIN_4
#define RED_3_GPIO_Port GPIOD
#define GREEN_3_Pin GPIO_PIN_5
#define GREEN_3_GPIO_Port GPIOD
#define BLUE_2_Pin GPIO_PIN_7
#define BLUE_2_GPIO_Port GPIOD
#define RED_2_Pin GPIO_PIN_9
#define RED_2_GPIO_Port GPIOG
#define GREEN_2_Pin GPIO_PIN_10
#define GREEN_2_GPIO_Port GPIOG
#define BLUE_1_Pin GPIO_PIN_11
#define BLUE_1_GPIO_Port GPIOG
#define RED_1_Pin GPIO_PIN_12
#define RED_1_GPIO_Port GPIOG
#define GREEN_1_Pin GPIO_PIN_13
#define GREEN_1_GPIO_Port GPIOG
/* USER CODE BEGIN Private defines */
#define AUDIO_BUFFER_SECS 3
#define EXT_PWR_DETECTED 	0b00000001
#define BATTERY_LOW			0b00000010
#define BATTERY_OVERCHRG	0b00000100
#define CHARGER_FAULT 		0b00001000
#define BATT_CHARGED		0b00010000
#define VBAT_LOW			0b00100000
#define VBAT_MISSING 		0b01000000

#define TS30    ((uint16_t*)((uint32_t)0x1FFF75A8))
#define TS130   ((uint16_t*)((uint32_t)0x1FFF75CA))


struct PowerStatus {
	uint8_t status_flag;
	uint16_t ureg_millivolts;
	uint16_t battery_millivolts;
	uint16_t vbat_millivolts;
};

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
