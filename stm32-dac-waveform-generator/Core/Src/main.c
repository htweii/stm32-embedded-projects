/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Switch_Button GPIO_PIN_13

#define Samples 20
#define TOTAL_WAVEFORM 7

#define DAC_LOW_VALUE 1.3
#define LOW_PERIODS 20
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac_ch1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/*** DAC settings ***/
float DAC_RANGE = 4095;
uint32_t DACData = 0;
uint16_t DACValues_Sawtooth[Samples];
uint16_t DACValues_Square[Samples];
uint16_t DACValues_Sine[Samples];
uint16_t DACValues_Trapezoidal[Samples];
uint16_t DACValues_HalfSine[Samples];
uint16_t DACValues_Triangle[Samples];
uint16_t DACValues_Noise[Samples];
uint16_t* DACValues[TOTAL_WAVEFORM] = {
	DACValues_Sawtooth,
	DACValues_Square,
	DACValues_Sine,
    DACValues_Trapezoidal,
	DACValues_HalfSine,
	DACValues_Triangle,
	DACValues_Noise
};

/*** Clear waveform residuals ***/
uint16_t DACValues_Low[Samples];
uint8_t Is_DACValues_Low = 0;
uint8_t low_period_count = 0;
uint8_t Current_Waveform = 0;
uint8_t Want_Switch = 0;
uint8_t Next_Waveform = 0;

/*** Button state settings and debounce ***/
uint8_t button_state = 1;      // 0: pressed, 1: released
uint8_t last_button_state = 1; // 0: pressed, 1: released
uint8_t button_pressed = 0;    // 0: not handled, 1: already handled

/*** Waveform lookup table ***/
float Waveform_Sawtooth[Samples];
float Waveform_Square[Samples];
float Waveform_Sine[Samples];
float Waveform_Trapezoidal[Samples];
float Waveform_HalfSine[Samples];
float Waveform_Triangle[Samples];
float Waveform_Noise[Samples];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac)
{
	if(Want_Switch)
	{
		Want_Switch = 0;
		low_period_count = 0;

		HAL_TIM_Base_Stop(&htim2);                     // Timer stop
		HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);       // DMA stop
		while((DMA1_Channel3->CCR & DMA_CCR_EN) != 0); // Wait for DMA stop

		HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)DACValues_Low, Samples ,DAC_ALIGN_12B_R); // Clear waveform residuals
		HAL_TIM_Base_Start(&htim2); // Timer restart

		Is_DACValues_Low = 1;
	}
	else if(Is_DACValues_Low)
	{
		low_period_count++;

        if(low_period_count < LOW_PERIODS)
		{
        	HAL_TIM_Base_Stop(&htim2);                     // Timer stop
        	HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);       // DMA stop
        	while((DMA1_Channel3->CCR & DMA_CCR_EN) != 0); // Wait for DMA stop

        	HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)DACValues_Low, Samples ,DAC_ALIGN_12B_R); // Clear waveform residuals
        	HAL_TIM_Base_Start(&htim2); // Timer restart
		}
		else
		{
			Is_DACValues_Low = 0;

			HAL_TIM_Base_Stop(&htim2);                     // Timer stop
        	HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);       // DMA stop
	        while((DMA1_Channel3->CCR & DMA_CCR_EN) != 0); // Wait for DMA stop

	    	HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)DACValues[Next_Waveform], Samples ,DAC_ALIGN_12B_R); // Switch to next waveform
	        HAL_TIM_Base_Start(&htim2); // Timer restart
		}
	}
}

void Generate_Waveforms(void)
{
	/*** Waveforms: Min voltage: 1.3V, Max voltage: 2.0V ***/

	// Sawtooth
	for(int i = 0; i < Samples; i++)
	{
		Waveform_Sawtooth[i] = 1.3 + 0.7 * i / (Samples - 1);
		DACValues_Sawtooth[i] = (uint16_t)(Waveform_Sawtooth[i] * DAC_RANGE / 3.3);
	}

	// Squarewave
	for(int i = 0; i < Samples; i++)
	{
		if(i < (Samples / 2))
			Waveform_Square[i] = 1.3;
		else
			Waveform_Square[i] = 2.0;
		DACValues_Square[i] = (uint16_t)(Waveform_Square[i] * DAC_RANGE / 3.3);
	}

	// Sinewave
	for(int i = 0; i < Samples; i++)
	{
		Waveform_Sine[i] = 1.65 + 0.35 * sin(2.0 * 3.141592 * i / Samples);
		DACValues_Sine[i] = (uint16_t)(Waveform_Sine[i] * DAC_RANGE / 3.3);
	}

	// Trapezoidal
	int rise = Samples * 0.2;
	int high = Samples * 0.5;
	int fall = Samples * 0.7;
	for(int i = 0; i < Samples; i++)
	{
		if(i < rise)
			Waveform_Trapezoidal[i] = 1.3 + 0.7 * i / rise;
		else if(i < high)
			Waveform_Trapezoidal[i] = 2.0;
		else if(i < fall)
			Waveform_Trapezoidal[i] = 2.0 - 0.7 * (i - high) / (fall - high);
		else
			Waveform_Trapezoidal[i] = 1.3;
		DACValues_Trapezoidal[i] = (uint16_t)(Waveform_Trapezoidal[i] * DAC_RANGE / 3.3);
	}

	// HalfSine
	for(int i = 0; i < Samples; i++)
	{
		Waveform_HalfSine[i] = 1.3 + 0.7 * sin(3.141592 * i / Samples);
		DACValues_HalfSine[i] = (uint16_t)(Waveform_HalfSine[i] * DAC_RANGE / 3.3);
	}

	// Triangle
	for(int i = 0; i < Samples; i++)
	{
		if(i < (Samples / 2))
			Waveform_Triangle[i] = 1.3 + 0.7 * (2.0 * i) / Samples;
		else
			Waveform_Triangle[i] = 2.0 - 0.7 * (2.0 * (i - Samples / 2)) / Samples;
		DACValues_Triangle[i] = (uint16_t)(Waveform_Triangle[i] * DAC_RANGE / 3.3);
	}

	// Noise
	for(int i = 0; i < Samples; i++)
	{
		float rand_value = (float)rand() / RAND_MAX;
		Waveform_Noise[i] = 1.3 + 0.7 * rand_value;
		DACValues_Noise[i] = (uint16_t)(Waveform_Noise[i] * DAC_RANGE / 3.3);
	}
}

void Generate_Low_Wave(void)
{
	for(int i = 0; i < Samples; i++)
		DACValues_Low[i] = (uint16_t)(DAC_LOW_VALUE * DAC_RANGE / 3.3);
}
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
  Generate_Waveforms();
  Generate_Low_Wave(); // Low Wave for clear waveform residuals
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_DAC_Init(&hdac1);
  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)DACValues[Current_Waveform], Samples ,DAC_ALIGN_12B_R);
  HAL_TIM_Base_Start(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  button_state = HAL_GPIO_ReadPin(GPIOC, Switch_Button); // Read button state

	  if(button_state == 0 && last_button_state == 1 && button_pressed == 0)
	  {
		  HAL_Delay(50); // Debounce delay
		  if(HAL_GPIO_ReadPin(GPIOC, Switch_Button) == 0)
		  {
			  Next_Waveform = (Current_Waveform + 1) % TOTAL_WAVEFORM; // Set next waveform
			  Current_Waveform = Next_Waveform;
			  Want_Switch = 1;
			  button_pressed = 1;
		  }
	  }
	  if(button_state == 1) // Allow next press
	  {
		  button_pressed = 0;
	  }

	  last_button_state = button_state;
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 3999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim2.Init.Period = 19;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
