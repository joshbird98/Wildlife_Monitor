/* Wildlife Monitor */
/**
  ******************************************************************************
  * @file           : main.cpp
  * @brief          : Main program body
  *****************************************************************************/

#include "main.h"
#include "fatfs.h"
#include "string.h"
#include "stdio.h"
#include "mcp33131.h"
#include "mcp41010.h"
#include "File_Handling.h"
#include "wav_header.h"
#include "SX1262.h"

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
COMP_HandleTypeDef hcomp1;
CRC_HandleTypeDef hcrc;
RTC_HandleTypeDef hrtc;
SD_HandleTypeDef hsd1;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim17;
UART_HandleTypeDef huart1;

struct PowerStatus PowerStatus1;
uint16_t mic_gain;
volatile uint32_t audio_buffer_location = 0;
volatile int16_t audio_buffer[AUDIO_SAMPLE_RATE * AUDIO_BUFFER_SECS];
volatile uint8_t audio_buffer_flag = 0;

char sd_buffer[100];

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_COMP1_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM17_Init(void);

static void Manual_TIM16(void);
struct PowerStatus updatePowerStatus(struct PowerStatus powerstat);
uint8_t write_audio_sample_sd(volatile int16_t *data, const char* animal_class, uint8_t prob, uint8_t pause);
void light_show(uint16_t delay, uint8_t repeats);
void ADC_Select_CH0 (void);
void ADC_Select_CH1 (void);
void ADC_Select_CHVBAT(void);
void printPowerStatus(struct PowerStatus power_status);
void startSleeping(void);
void preprocess_audio(uint32_t sample_start_location);
void setRx(void);
void setTx(void);
uint16_t auto_adjust_gain(uint32_t sample_start_location);

int main(void)
{
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_COMP1_Init();
	MX_SDMMC1_SD_Init();
	MX_SPI1_Init();
	MX_USART1_UART_Init();
	MX_FATFS_Init();
	MX_CRC_Init();
	MX_RTC_Init();
	Manual_TIM16();
	MX_TIM17_Init();

	HAL_COMP_Start_IT(&hcomp1);
	light_show(200, 1);
	println("Wildlife monitor has started up!");

	/* Infinite loop */
	while (1)
	{
		if (audio_buffer_flag == 1)	//audio buffer is half full with new data
		{
			light_show(10,1);
			audio_buffer_flag = 0;

			// determining start location of audio sample in buffer (either 0 or full/2)
			// determine which half to the buffer to look at (start->halfway OR halfway->end)
			uint32_t sample_start_location = 0;
			if (audio_buffer_location < (AUDIO_SAMPLE_RATE * AUDIO_SAMPLE_SECS))
			{
				sample_start_location = (AUDIO_SAMPLE_RATE * AUDIO_SAMPLE_SECS);
			}

			//do some pre-processing to remove pops
			preprocess_audio(sample_start_location);

			//save newest audio sample to SD card

			const char *animal_class = "FOX";
			uint8_t prob = 81;
			write_audio_sample_sd(&audio_buffer[sample_start_location], animal_class, prob, 1);

			/*// Print out the entire sample for analysis on PC (blocking process)
			println("NEW_SAMPLE");
			__HAL_RCC_TIM16_CLK_DISABLE(); // stop clock from interrupting dump
			for (uint32_t i = sample_start_location; i < (sample_start_location + (AUDIO_SAMPLE_RATE * AUDIO_SAMPLE_SECS)); i++)
			{
				printint16_t(audio_buffer[i]);
				print(",");
			}
			println("");
			__HAL_RCC_TIM16_CLK_ENABLE();

			light_show(200,1);

			//mic_gain = auto_adjust_gain(sample_start_location);*/
		}
	}
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
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

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

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x7;
  sTime.Minutes = 0x30;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_DECEMBER;
  sDate.Date = 0x10;
  sDate.Year = 0x21;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 250;
  hsd1.Init.Transceiver = SDMMC_TRANSCEIVER_DISABLE;
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
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
}


/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 0;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 150-1;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
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
  //HAL_GPIO_WritePin(RADIO_DIO1_GPIO_Port, RADIO_DIO1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RADIO_RST_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CS_RADIO_GPIO_Port, CS_RADIO_Pin, GPIO_PIN_SET);

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
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  //GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
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

static void Manual_TIM16(void) {
	__HAL_RCC_TIM16_CLK_ENABLE();
	TIM16 -> PSC = (1 - 1);
	TIM16 -> ARR = ((120000000 / AUDIO_SAMPLE_RATE) - 1);

	TIM16 -> EGR |= TIM_EGR_UG;  // reinitialize the counter and reload registers
	TIM16 -> DIER |= TIM_DIER_UIE;

	NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);

	TIM16 -> CR1 = TIM_CR1_CEN;
}

//if you compile as C++ you need to declare handlers as "normal" C functions
//#ifdef`s are not needed if this code will never be compiled as C progream
#ifdef __cplusplus
extern "C" {
#endif

void TIM1_UP_TIM16_IRQHandler(void)
{
    if(TIM16 -> SR & TIM_SR_UIF)
    {
        TIM16 -> SR = ~(TIM_SR_UIF); // clear UIF flag
        //sample audio on this interrupt
		audio_buffer[audio_buffer_location] = (int16_t)mcp33131_get16b(hspi1);

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

#ifdef __cplusplus
}
#endif


// when the audio input rises above 3/4Ref, this interrupt function runs
// If microphone gain is set to maximum, the device could sleep until this interrupt detects noise
void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp)
{
  if (hcomp == &hcomp1)
  {
	// comp interrupt mode will only have been activated if device has been sent to sleep
	// therefore, turn on other interrupts and tick etc, then disable comp interrupt mode.
    HAL_ResumeTick();
    __HAL_RCC_TIM16_CLK_ENABLE();
    hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
    HAL_COMP_Init(&hcomp1);
  }
}


// this sends the device to a low-power sleep mode to conserve battery
// it wakes up when the microphone detects noise, via a comparator interrupt
// This could be used when no audio is detected for a while
void startSleeping(void)
{
  set_mic_gain(2000, hspi1);	// set microphone gain (determines level of noise needed to wake device up)
  //HAL_TIM_Base_Stop(&htim16);	// turn off timer interrupts
  __HAL_RCC_TIM16_CLK_DISABLE();
  HAL_SuspendTick();			// turn off systick interrupt
  hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING;	//setup comparator for interrupt wakeup
  HAL_COMP_Init(&hcomp1);		// turn on comparator
  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);	//go to sleep
  //Interrupt from comparator will wake CPU, and should handle turning back on ticks and timer interrupt
}


// Function updates the power status structure, with newest voltages and status's
// Note that I don't think VBAT reading is actually correct... will need to check ADC config
// example usage
//PowerStatus1 = updatePowerStatus(PowerStatus1);
//printPowerStatus(PowerStatus1);
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
uint8_t write_audio_sample_sd(volatile int16_t *data, const char* animal_class, uint8_t prob, uint8_t pause)
{
	// read the RTC registers inside the STM32
	uint32_t date_reg = RTC->DR;
	uint32_t time_reg = RTC->TR;
	// interpret their BCD format into regular values and assign to month,day,hours,min,secs
	uint8_t day = (10 * ((date_reg >> 4	) & 0x3)) + (date_reg & 0x3F);
	uint8_t month = (10 * ((date_reg >> 12) & 0x1)) + ((date_reg >> 8) & 0xF);
	uint8_t year = (10 * ((date_reg >> 20) & 0xF)) + ((date_reg >> 16) & 0xF);
	uint8_t hours = (10 * ((time_reg >> 20) & 0x3)) + ((time_reg >> 16) & 0xF);
	uint8_t mins = (10 * ((time_reg >> 12) & 0x7)) + ((time_reg >> 8) & 0xF);
	uint8_t secs = (10 * ((time_reg >> 4) & 0x7)) + (time_reg & 0xF);
	if (pause == 1) __HAL_RCC_TIM16_CLK_DISABLE(); // stop clock from interrupting dump
	if (Mount_SD("/")){
		println("Failed to write audio sample onto SD card.");
		return 0;
	}

	uint32_t entry_number = entry_number_update();
	if (entry_number == 0) return 0;	//failure to complete - check SD card?

	char new_audio_filename[100];
	sprintf(new_audio_filename, "%08lu.WAV", entry_number);
	Create_File(new_audio_filename);
	Write_File_u8(new_audio_filename, wav_header, 44);
	Update_File_16(new_audio_filename, data, (AUDIO_SAMPLE_RATE*AUDIO_BUFFER_SECS));

	//create matching metadata for wav file
	char new_metadata_filename[100];
	sprintf(new_metadata_filename, "%08lu.TXT", entry_number);
	Create_File(new_metadata_filename);
	char buffer[100];
	sprintf(buffer, "METADATA for %08lu.WAV\nCLASSIFICATION: %s\nPROBABILITY: %d\nDATE: %d/%d/%d\nTIME: %d:%d:%d",
			entry_number, animal_class, prob, day, month, year, hours, mins, secs);
	Update_File(new_metadata_filename, buffer);

	Unmount_SD("/");
	char print_buffer[100];
	sprintf(print_buffer, "%s sample recorded on %d/%d/%d at %d:%d:%d", animal_class, day, month, year, hours, mins, secs);
	println(print_buffer);
	if (pause == 1) __HAL_RCC_TIM16_CLK_ENABLE();
	return 1;
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
	  HAL_GPIO_WritePin(RED_1_GPIO_Port, RED_1_Pin, (GPIO_PinState) !red);
	  HAL_GPIO_WritePin(GREEN_1_GPIO_Port, GREEN_1_Pin, (GPIO_PinState) !green);
	  HAL_GPIO_WritePin(BLUE_1_GPIO_Port, BLUE_1_Pin, (GPIO_PinState) !blue);
	  break;
	case 2:
	  HAL_GPIO_WritePin(RED_2_GPIO_Port, RED_2_Pin, (GPIO_PinState) !red);
	  HAL_GPIO_WritePin(GREEN_2_GPIO_Port, GREEN_2_Pin, (GPIO_PinState) !green);
	  HAL_GPIO_WritePin(BLUE_2_GPIO_Port, BLUE_2_Pin, (GPIO_PinState) !blue);
	  break;
	case 3:
	  HAL_GPIO_WritePin(RED_3_GPIO_Port, RED_3_Pin, (GPIO_PinState) !red);
	  HAL_GPIO_WritePin(GREEN_3_GPIO_Port, GREEN_3_Pin, (GPIO_PinState) !green);
	  HAL_GPIO_WritePin(BLUE_3_GPIO_Port, BLUE_3_Pin, (GPIO_PinState) !blue);
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
void print(const char _out[])
{
  HAL_UART_Transmit(&huart1, (uint8_t *) _out, strlen(_out), 10);
}


// Prints out a string to serial, and starts a newline. Use as print("Hello!");
void println(const char _out[])
{
  HAL_UART_Transmit(&huart1, (uint8_t *) _out, strlen(_out), 10);
  char newline[2];
  newline[0] = '\r';
  newline[1] = '\n';
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


void printint32_t(int32_t value)
{
  char str[10];
  sprintf(str, "%ld", value);
  print(str);
}


void printint32_tln(int32_t value)
{
  char str[10];
  sprintf(str, "%ld", value);
  print(str);
  char newline[2];
  newline[0] = '\r';
  newline[1] = '\n';
  HAL_UART_Transmit(&huart1, (uint8_t *) newline, 2, 10);
}


//Print raw uint16_t
void print_raw_uint16_t(uint16_t value)
{
	HAL_UART_Transmit(&huart1, (uint8_t*)(&value), 2, 100);
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


// Prints out a uint8_t type to serial
// uint8_t x = 42; printuint16_t(x);
void printint8_t(int8_t value)
{
  char str[10];
  sprintf(str, "%d", value);
  print(str);
}


void print_lora_pkt(void)
{
  HAL_UART_Transmit(&huart1, (uint8_t *) recMessage, LORA_MAX_PAYLOAD + 3, 10);
  char newline[2];
  newline[0] = '\r';
  newline[1] = '\n';
  HAL_UART_Transmit(&huart1, (uint8_t *) newline, 2, 10);
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


// This function attempts to automatically adjust the microphone gain
// based on the loudness of a previous audio sample
// ARG: uint32_t sample_start_location tells the function where in audio_buffer to look
// RET: uint16_t mic_gain returns the new value of the microphone gain, following adjustment
uint16_t auto_adjust_gain(uint32_t sample_start_location) {
	//determine max and min audio levels from last sample
	uint16_t max_value = 32767;
	uint16_t min_value= 32767;
	for (uint32_t i = 0; i < (AUDIO_SAMPLE_RATE * AUDIO_BUFFER_SECS / 2); i++) {
		if (audio_buffer[sample_start_location + i] > max_value) max_value = audio_buffer[sample_start_location + i];
		if (audio_buffer[sample_start_location + i] < min_value) min_value = audio_buffer[sample_start_location + i];
	}
	// now adjust microphone gain to attempt to get reach target audio range
	uint16_t range = max_value - min_value;
	mic_gain = set_mic_gain(((mic_gain *  TARGET_BIT_RANGE) / range), hspi1);
	return mic_gain;
}


// calculates the average of an audio sample (for use with correcting future samples
// also removes any pops from sample, hopefully
void preprocess_audio(uint32_t sample_start_location)
{
	int32_t sample_sum = 0;
	int32_t sample_count = 0;
	int16_t sample_average;

	for (uint32_t i = sample_start_location; i < sample_start_location + (AUDIO_SAMPLE_RATE * AUDIO_SAMPLE_SECS); i++)
	{
		sample_sum += audio_buffer[i];
		sample_count += 1;
	}
	sample_average = (int16_t)(sample_sum / sample_count);


	for (uint32_t i = sample_start_location + 2; i < (sample_start_location + (AUDIO_SAMPLE_RATE * AUDIO_SAMPLE_SECS) - 2); i++)
	{
		audio_buffer[i] = audio_buffer[i] - sample_average;
	}
}


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
  println("Assert failed! Wrong parameters value: file %s on line %d\r\n", file, line);
}
#endif /* USE_FULL_ASSERT */


// this code can be used as a LED heartbeat, but also to send the device to sleep every 5 seconds, until a loud noise wakes it up
/*uint32_t new_time = HAL_GetTick();
	if ((new_time - time1) > 500)		// every 500 ms, flash the lights like a heartbeat
	{
		time1 = new_time;	// update time of last heartbeat
		light_show(5, 1);   // heartbeat!
	}
	if ((new_time - time2) > 5000)		// every 5000ms, send the device to sleep (wakes up on audio detection)
	{
		time2 = new_time;	// update time
		startSleeping();	// time for bed!
	}*/

