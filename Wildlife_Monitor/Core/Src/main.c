/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "mcp33131.h"
#include "mcp41010.h"
#include "File_Handling.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

COMP_HandleTypeDef hcomp1;

CRC_HandleTypeDef hcrc;

DFSDM_Channel_HandleTypeDef hdfsdm1_channel1;

RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
struct PowerStatus PowerStatus1;


//Audio capture - not yet tested
// TIMER16 sets off an interrupt every 1/AUDIO_SAMPLE_RATE seconds.
// On this interrupt, a function retrieves a 16-bit audio sample from
// the external MCP33131 ADC, and stores it in the audio_buffer below.
// Then the audio buffer location is incremented, maybe rolling back
// to position zero. This buffer holds AUDIO_BUFFER_SECS seconds of audio.
uint32_t audio_buffer_location = 0;
uint16_t audio_buffer[AUDIO_SAMPLE_RATE * AUDIO_BUFFER_SECS];
// When buffer location reaches 50% or 100%, an interrupt sets a flag
uint8_t audio_buffer_flag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_COMP1_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */
struct PowerStatus updatePowerStatus(struct PowerStatus powerstat);
void write_audio_sample_sd(uint16_t *data);
void led_rgb(uint8_t led, uint8_t colour);
void light_show(uint16_t delay, uint8_t repeats);
void ADC_Select_CH0 (void);
void ADC_Select_CH1 (void);
void ADC_Select_CHVBAT(void);
void printPowerStatus(struct PowerStatus power_status);
void startSleeping(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_COMP1_Init();
  MX_DFSDM1_Init();
  MX_SDMMC1_SD_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_FATFS_Init();
  MX_CRC_Init();
  MX_RTC_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */

  //the following code is just intended as examples to show how some in-built or custom functions COULD be used.
  //feel free to change anything, and apologies for poor documentation, message me anytime for clarifications - JB

  HAL_TIM_Base_Start_IT(&htim16);
  HAL_COMP_Start_IT(&hcomp1);

  light_show(200, 1);
  println("Wildlife monitor has started up!");

  PowerStatus1 = updatePowerStatus(PowerStatus1);
  printPowerStatus(PowerStatus1);

  set_mic_gain(1000, hspi1); // initialises the microphone gain to 1000

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint32_t time1 = HAL_GetTick();	// two variables for tracking time, used for some non-blocking periodic events in main()
  uint32_t time2 = time1;

  while (1)
  {
	uint32_t new_time = HAL_GetTick();
	if ((new_time - time1) > 500)		// every 500 ms, flash the lights like a heartbeat
	{
		time1 = new_time;	// update time of last heartbeat
		light_show(5, 1);   // heartbeat!
	}
	if ((new_time - time2) > 5000)		// every 5000ms, send the device to sleep (wakes up on audio detection)
	{
		time2 = new_time;	// update time
		startSleeping();	// time for bed!
	}/*

	// TESTING THE AUDIO BUFFER SYSTEM - seems okay?
	//Check audio buffer for new sample
	if (audio_buffer_flag == 1) {	//buffer is half full with new data
		light_show(5,1);
		audio_buffer_flag = 0;
		// determining start location of audio sample in buffer (either 0 or full/2)
		uint32_t sample_start_location = 0;
		if (audio_buffer_location < (AUDIO_SAMPLE_RATE * AUDIO_BUFFER_SECS / 2)) {
			sample_start_location = (AUDIO_SAMPLE_RATE*AUDIO_BUFFER_SECS / 2);
		}
		//complete sample is briefly stored in memory from...
		//audio_buffer[sample_start_location] to audio_buffer[sample_start_location+(AUDIO_SAMPLE_RATE * AUDIO_BUFFER_SECS / 2)]

		//save newest audio sample to SD card
		//write_audio_sample_sd(&audio_buffer[sample_start_location]);
		println("AUDIO CAPTURE... ");
		for (uint8_t i = 0; i < 10; i++)
		{
			print("audio_buffer[");
			printuint32_t(sample_start_location + i);
			print("] = ");
			printuint16_t(audio_buffer[sample_start_location + i]);
			println("");
		}
		println("");
	}*/

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 30;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief COMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP1_Init(void)
{

  /* USER CODE BEGIN COMP1_Init 0 */

  /* USER CODE END COMP1_Init 0 */

  /* USER CODE BEGIN COMP1_Init 1 */

  /* USER CODE END COMP1_Init 1 */
  hcomp1.Instance = COMP1;
  hcomp1.Init.InvertingInput = COMP_INPUT_MINUS_3_4VREFINT;
  hcomp1.Init.NonInvertingInput = COMP_INPUT_PLUS_IO2;
  hcomp1.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp1.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp1.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp1.Init.Mode = COMP_POWERMODE_HIGHSPEED;
  hcomp1.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING;
  if (HAL_COMP_Init(&hcomp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP1_Init 2 */

  /* USER CODE END COMP1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_channel1.Instance = DFSDM1_Channel1;
  hdfsdm1_channel1.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel1.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel1.Init.OutputClock.Divider = 2;
  hdfsdm1_channel1.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel1.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel1.Init.Input.Pins = DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS;
  hdfsdm1_channel1.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel1.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel1.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel1.Init.Awd.Oversampling = 1;
  hdfsdm1_channel1.Init.Offset = 0;
  hdfsdm1_channel1.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 0;
  hsd1.Init.Transceiver = SDMMC_TRANSCEIVER_DISABLE;
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = (120000000/AUDIO_SAMPLE_RATE)-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 0;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 90-1;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, CS_EINK_Pin|DC_EINK_Pin|RST_EINK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RADIO_TX_GPIO_Port, RADIO_TX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_MIC_GAIN_GPIO_Port, CS_MIC_GAIN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_MIC_ADC_GPIO_Port, CS_MIC_ADC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RADIO_DIO1_GPIO_Port, RADIO_DIO1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RADIO_RST_Pin|CS_RADIO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, BLUE_3_Pin|RED_3_Pin|GREEN_3_Pin|BLUE_2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, RED_2_Pin|GREEN_2_Pin|BLUE_1_Pin|RED_1_Pin
                          |GREEN_1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : CHRG_PG_Pin CHRG_STAT2_Pin CHRG_STAT1_Pin */
  GPIO_InitStruct.Pin = CHRG_PG_Pin|CHRG_STAT2_Pin|CHRG_STAT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_EINK_Pin DC_EINK_Pin RST_EINK_Pin */
  GPIO_InitStruct.Pin = CS_EINK_Pin|DC_EINK_Pin|RST_EINK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : BUSY_EINK_Pin */
  GPIO_InitStruct.Pin = BUSY_EINK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUSY_EINK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RADIO_TX_Pin */
  GPIO_InitStruct.Pin = RADIO_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RADIO_TX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RADIO_RX_Pin */
  GPIO_InitStruct.Pin = RADIO_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RADIO_RX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MODE_Pin RADIO_BUSY_Pin CARD_Pin */
  GPIO_InitStruct.Pin = MODE_Pin|RADIO_BUSY_Pin|CARD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_MIC_GAIN_Pin CS_MIC_ADC_Pin */
  GPIO_InitStruct.Pin = CS_MIC_GAIN_Pin|CS_MIC_ADC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RADIO_DIO1_Pin */
  GPIO_InitStruct.Pin = RADIO_DIO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RADIO_DIO1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RADIO_RST_Pin CS_RADIO_Pin */
  GPIO_InitStruct.Pin = RADIO_RST_Pin|CS_RADIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BLUE_3_Pin RED_3_Pin GREEN_3_Pin BLUE_2_Pin */
  GPIO_InitStruct.Pin = BLUE_3_Pin|RED_3_Pin|GREEN_3_Pin|BLUE_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : RED_2_Pin GREEN_2_Pin BLUE_1_Pin RED_1_Pin
                           GREEN_1_Pin */
  GPIO_InitStruct.Pin = RED_2_Pin|GREEN_2_Pin|BLUE_1_Pin|RED_1_Pin
                          |GREEN_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

// Callback: timer has rolled over - time to sample audio from via external ADC
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check timer interrupt came from TIM16
  if (htim == &htim16)
  {
    //sample audio on this interrupt
	audio_buffer[audio_buffer_location] = 0x12;
	audio_buffer_location += 1;
	if (audio_buffer_location >= (AUDIO_SAMPLE_RATE * AUDIO_BUFFER_SECS)) {
		audio_buffer_location = 0; //handles rollover (circular buffer)
		audio_buffer_flag = 1;
	}
	if (audio_buffer_location == (AUDIO_SAMPLE_RATE * AUDIO_BUFFER_SECS / 2)) {
		audio_buffer_flag = 1;
	}
  }
}

// when the audio input rises above 3/4Ref, this interrupt function runs
// If microphone gain is set to maximum, the device could sleep until this interrupt detects noise
void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp)
{
  if (hcomp == &hcomp1)
  {
	// comp interrupt mode will only have been activated if device has been sent to sleep
	// therefore, turn on other interrupts and tick etc, then disable comp interrupt mode.
    HAL_ResumeTick();
    HAL_TIM_Base_Stop(&htim16);
    hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
    HAL_COMP_Init(&hcomp1);
  }
}

// this sends the device to a low-power sleep mode to conserve battery
// it wakes up when the microphone detects noise, via a comparator interrupt
// This could be used when no audio is detected for a while
void startSleeping(void)
{
  set_mic_gain(2000, hspi1);			// set microphone gain (determines level of noise needed to wake device up)
  HAL_TIM_Base_Stop(&htim16);	// turn off timer interrupts
  HAL_SuspendTick();			// turn off systick interrupt
  hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING;	//setup comparator for interrupt wakeup
  HAL_COMP_Init(&hcomp1);		// turn on comparator
  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);	//go to sleep
  //Interrupt from comp will wake CPU, and should handle turning back on ticks and timer interrupt
}
// Function updates the power status structure, with newest voltages and status's
// Note that I don't think VBAT reading is actually correct... will need to check ADC config
struct PowerStatus updatePowerStatus(struct PowerStatus powerstat)
{
	//get adc results from three separate channels...
	uint16_t adc_result;
	ADC_Select_CH0();
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 1000);
    adc_result = HAL_ADC_GetValue(&hadc1);
    powerstat.battery_millivolts = adc_result * 1.611721;
    HAL_ADC_Stop(&hadc1);

    ADC_Select_CH1();
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 1000);
    adc_result = HAL_ADC_GetValue(&hadc1);
    powerstat.ureg_millivolts = adc_result * 1.611721;
    HAL_ADC_Stop(&hadc1);

    ADC_Select_CHVBAT();
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 1000);
    adc_result = HAL_ADC_GetValue(&hadc1);
    powerstat.vbat_millivolts = adc_result * 1.611721;
    HAL_ADC_Stop(&hadc1);

    // interpret analog voltages, and set flags accordingly
	if (powerstat.ureg_millivolts > (powerstat.battery_millivolts + 200)) 	//solar or usb is plugged in
		powerstat.status_flag |= EXT_PWR_DETECTED;
	else
		powerstat.status_flag &= ~EXT_PWR_DETECTED;

	if (powerstat.battery_millivolts < 3300)								//battery is running low!
		powerstat.status_flag |= BATTERY_LOW;
	else
		powerstat.status_flag &= ~BATTERY_LOW;

	if (powerstat.battery_millivolts > 4300)								//battery appears overcharged (>4.3V)
		powerstat.status_flag |= BATTERY_OVERCHRG;
	else
		powerstat.status_flag &= ~BATTERY_OVERCHRG;

	if (powerstat.vbat_millivolts < 2700)
		powerstat.status_flag |= VBAT_LOW;									//backup cell for RTC is low!
	else
		powerstat.status_flag &= ~VBAT_LOW;

	if (powerstat.vbat_millivolts < 1800)									//backup cell for RTC is not detected/inserted
	{
		powerstat.status_flag |= VBAT_MISSING;
		powerstat.status_flag &= ~VBAT_LOW;
	}
	else
		powerstat.status_flag &= ~VBAT_MISSING;

	// now read from MCP73871 digital outputs for charger-status
	uint8_t chrg_stat1 = HAL_GPIO_ReadPin(CHRG_STAT1_GPIO_Port, CHRG_STAT1_Pin);
	uint8_t chrg_stat2 = HAL_GPIO_ReadPin(CHRG_STAT2_GPIO_Port, CHRG_STAT2_Pin);
	uint8_t chrg_pg    = HAL_GPIO_ReadPin(CHRG_PG_GPIO_Port, CHRG_PG_Pin);

	// interpret the charger status pins, and set flags accordingly
	if ((chrg_stat2 == 0) && (chrg_pg == 0))
	{
		if (chrg_stat1 == 0)
		{
			powerstat.status_flag |= CHARGER_FAULT;
			powerstat.status_flag &= ~BATT_CHARGED;
		}
		else
		{
			powerstat.status_flag |= BATT_CHARGED;
			powerstat.status_flag &= ~CHARGER_FAULT;
		}
	}

	return powerstat;
}

// Writes in half the audio buffer to the SD in a file with timestamped name - UNTESTED
void write_audio_sample_sd(uint16_t *data)
{
	// read the RTC registers inside the STM32
	uint32_t date_reg = RTC->DR;
	uint32_t time_reg = RTC->TR;
	// interpret their BCD format into regular values and assign to month,day,hours,min,secs
	uint8_t month = (10 * ((date_reg >> 12) & 0x1)) + ((date_reg >> 8) & 0xF);
	uint8_t day = (10 * ((date_reg >> 4	) & 0x3)) + (date_reg & 0x3F);
	uint8_t hours = (10 * ((time_reg >> 20) & 0x3)) + ((time_reg >> 16) & 0xF);
	uint8_t mins = (10 * ((time_reg >> 12) & 0x7)) + ((time_reg >> 8) & 0xF);
	uint8_t secs = (10 * ((time_reg >> 4) & 0x7)) + (time_reg & 0xF);

	Mount_SD("/");
	char filename[100];
	sprintf(filename, "AUDIO_SAMPLE_%d-%d_%d%d%d.raw", month, day, hours, mins, secs);
	Create_File(filename);
	Write_File_u16(filename, data, (AUDIO_SAMPLE_RATE*AUDIO_BUFFER_SECS));
	Unmount_SD("/");
}

// Set the led (1, 2 or 3) to a specific colour.
// Available colours are (r)ed, (g)reen, (b)lue, (y)ellow, (p)urple, (c)yan, (w)hite or (o)ff.
// Use as led_rgb(2, 'r');
void led_rgb(uint8_t led, uint8_t colour)
{
  uint8_t red = 0;
  uint8_t green = 0;
  uint8_t blue = 0;
  switch (colour)
  {
	case 'r':	//red
	  red = 1;
	  break;
	case 'g':	//green
	  green = 1;
	  break;
	case 'b':	//blue
	  blue = 1;
	  break;
	case 'y':	//yellow
	  red = 1;
	  green = 1;
	  break;
	case 'p':	//purple
	  red = 1;
	  blue = 1;
	  break;
	case 'c':	//cyan or teal
	  green = 1;
	  blue = 1;
	  break;
	case 'w':	//white
	  red = 1;
	  green = 1;
	  blue = 1;
	  break;
	case 'o':	//off
	  break;
	default:
	  break;
  }

  switch (led)
  {
	case 1:
	  HAL_GPIO_WritePin(RED_1_GPIO_Port, RED_1_Pin, !red);
	  HAL_GPIO_WritePin(GREEN_1_GPIO_Port, GREEN_1_Pin, !green);
	  HAL_GPIO_WritePin(BLUE_1_GPIO_Port, BLUE_1_Pin, !blue);
	  break;
	case 2:
	  HAL_GPIO_WritePin(RED_2_GPIO_Port, RED_2_Pin, !red);
	  HAL_GPIO_WritePin(GREEN_2_GPIO_Port, GREEN_2_Pin, !green);
	  HAL_GPIO_WritePin(BLUE_2_GPIO_Port, BLUE_2_Pin, !blue);
	  break;
	case 3:
	  HAL_GPIO_WritePin(RED_3_GPIO_Port, RED_3_Pin, !red);
	  HAL_GPIO_WritePin(GREEN_3_GPIO_Port, GREEN_3_Pin, !green);
	  HAL_GPIO_WritePin(BLUE_3_GPIO_Port, BLUE_3_Pin, !blue);
	  break;
	default:
	  break;
  }
}

// Flashes all the RGB lights. Use as light_show(100, 5);
void light_show(uint16_t delay, uint8_t repeats)
{
  while (repeats > 0)
  {
	led_rgb(1,'p');
	led_rgb(2,'o');
	led_rgb(3,'o');
	HAL_Delay(delay);
	led_rgb(1,'r');
	led_rgb(2,'p');
	led_rgb(3,'o');
	HAL_Delay(delay);
	led_rgb(1,'y');
	led_rgb(2,'r');
	led_rgb(3,'p');
	HAL_Delay(delay);
	led_rgb(1,'g');
	led_rgb(2,'y');
	led_rgb(3,'r');
	HAL_Delay(delay);
	led_rgb(1,'b');
	led_rgb(2,'g');
	led_rgb(3,'y');
	HAL_Delay(delay);
	led_rgb(1,'c');
	led_rgb(2,'b');
	led_rgb(3,'g');
	HAL_Delay(delay);
	led_rgb(1,'w');
	led_rgb(2,'c');
	led_rgb(3,'b');
	HAL_Delay(delay);
	led_rgb(1,'o');
	led_rgb(2,'w');
	led_rgb(3,'c');
	HAL_Delay(delay);
	led_rgb(1,'o');
	led_rgb(2,'o');
	led_rgb(3,'w');
	HAL_Delay(delay);
	led_rgb(1,'o');
    led_rgb(2,'o');
	led_rgb(3,'o');
	HAL_Delay(delay);
	repeats -= 1;
  }
}

// Prints out a string to serial. Use as print("Hello, world.")
void print(char _out[])
{
  HAL_UART_Transmit(&huart1, (uint8_t *) _out, strlen(_out), 10);
}

// Prints out a string to serial, and starts a newline. Use as print("Hello!");
void println(char _out[])
{
  HAL_UART_Transmit(&huart1, (uint8_t *) _out, strlen(_out), 10);
  char newline[2] = "\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t *) newline, 2, 10);
}

// Prints out a uint16_t type to serial.
// uint16_t x = 3454242; printuint16_t(x);
void printuint32_t(uint32_t value)
{
  char str[10];
  sprintf(str, "%lu", value);
  print(str);
}

// Prints out a uint16_t type to serial.
// uint16_t x = 4242; printuint16_t(x);
void printuint16_t(uint16_t value)
{
  char str[10];
  sprintf(str, "%u", value);
  print(str);
}

// Prints out a uint16_t type to serial.
// int16_t x = -4242; printint16_t(x);
void printint16_t(int16_t value)
{
  char str[10];
  sprintf(str, "%d", value);
  print(str);
}

// Prints out a uint8_t type to serial
// uint8_t x = 42; printuint16_t(x);
void printuint8_t(uint8_t value)
{
  char str[10];
  sprintf(str, "%u", value);
  print(str);
}

// Set the ADC Channel
void ADC_Select_CH0 (void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

// Set the ADC Channel
void ADC_Select_CH1 (void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

// Set the ADC Channel
void ADC_Select_CHVBAT (void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_VBAT;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

// Prints out a bunch of information over serial.
// Shows power voltages and other power information.
void printPowerStatus(struct PowerStatus power_status)
{
	println("Power Info: ");
	print("BATTERY VOLTAGE: ");
	printuint16_t(power_status.battery_millivolts);
	println(" mV");
    print("UNREGULATED VOLTAGE: ");
    printuint16_t(power_status.ureg_millivolts);
    println(" mV");
    print("BACKUP BATTERY VOLTAGE: ");
    printuint16_t(power_status.vbat_millivolts);
    println(" mV");
    print("STATUS: ");
    printuint8_t(power_status.status_flag);
    println("");
    if ((power_status.status_flag & EXT_PWR_DETECTED) == EXT_PWR_DETECTED) println("External power connected.");
    if ((power_status.status_flag & BATTERY_LOW) == BATTERY_LOW) println("Battery power low.");
    if ((power_status.status_flag & BATTERY_OVERCHRG) == BATTERY_OVERCHRG) println("Battery overcharged... run!");
    if ((power_status.status_flag & CHARGER_FAULT) == CHARGER_FAULT) println("Charger fault.");
    if ((power_status.status_flag & BATT_CHARGED) == BATT_CHARGED) println("Battery fully charged.");
    if ((power_status.status_flag & VBAT_LOW) == VBAT_LOW) println("Backup battery power low.");
    if ((power_status.status_flag & VBAT_MISSING) == VBAT_MISSING) println("Backup battery missing.");

    println("");
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
	  led_rgb(1, 'r');
	  led_rgb(2, 'r');
	  led_rgb(3, 'r');
	  HAL_Delay(250);
	  led_rgb(1, 'o');
	  led_rgb(2, 'o');
	  led_rgb(3, 'o');
	  HAL_Delay(250);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

//just trying out SD stuff here
//Mount_SD("/");
//Format_SD();
//Create_File("FILE1.TXT");
//Unmount_SD("/");
//uint8_t indx = 42;
//char sd_writebuffer[100];
//Mount_SD("/");
//sprintf(sd_writebuffer, "Hello ---> %d\n", indx);
//Update_File("FILE1.TXT", sd_writebuffer);
//Unmount_SD("/");*/

/* TESTING THE AUDIO BUFFER SYSTEM - seems okay?
//Check audio buffer for new sample
	if (audio_buffer_flag == 1) {	//buffer is half full with new data
		audio_buffer_flag = 0;
		// determining start location of audio sample in buffer (either 0 or full/2)
		uint32_t sample_start_location = 0;
		if (audio_buffer_location < (AUDIO_SAMPLE_RATE * AUDIO_BUFFER_SECS / 2)) {
			sample_start_location = (AUDIO_SAMPLE_RATE*AUDIO_BUFFER_SECS / 2);
		}
		//complete sample is briefly stored in memory from...
		//audio_buffer[sample_start_location] to audio_buffer[sample_start_location+(AUDIO_SAMPLE_RATE * AUDIO_BUFFER_SECS / 2)]

		//save newest audio sample to SD card
		//write_audio_sample_sd(&audio_buffer[sample_start_location]);
		println("AUDIO CAPTURE... ");
		for (uint8_t i = 0; i < 10; i++)
		{
			print("audio_buffer[");
			printuint32_t(sample_start_location + i);
			print("] = ");
			printuint16_t(audio_buffer[sample_start_location + i]);
			println("");
		}
		println("");
	}*/
