
#include "main.h"

ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);

void test_start(void);
void test_testing(void);
void banner(void);



uint32_t time;
uint16_t adc_value;


typedef struct
{
	uint16_t time;
	
}Count;
Count count;


typedef struct
{
	uint8_t test;
	uint8_t display;
}Status;
Status status;

int main(void)
{
  HAL_Init();

  SystemClock_Config();


  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
	
	delay_init();
	
	I2C_Init();
	
	HAL_Delay(1000);
	LCD_ON;
	HAL_Delay(1000);
	
	/*Write_String(0, GREEN_C, "MAIN_1 - UB_1 : IN 3.3[V] - OUT 3.3[V]:OK");
			Write_String(1, GREEN_C, "MAIN_2 - UB_2 : IN 3.3[V] - OUT 3.3[V]:OK");
			Write_String(2, GREEN_C, "MAIN_4 - UB_4 : IN 3.3[V] - OUT 3.3[V]:OK");
			Write_String(3, GREEN_C, "MAIN_5 - UB_5 : IN 3.3[V] - OUT 3.3[V]:OK");
			Write_String(4, GREEN_C, "MAIN_6 - UB_6 : IN 3.3[V] - OUT 3.3[V]:OK");
			Write_String(5, GREEN_C, "MAIN_7 - UB_7 : IN 3.3[V] - OUT 3.3[V]:OK");
			Write_String(6, GREEN_C, "MAIN_8 - UB_8 : IN 3.3[V] - OUT 3.3[V]:OK");
			Write_String(7, GREEN_C, "MAIN_9 - UB_9 : IN 3.3[V] - OUT 3.3[V]:OK");
			Write_String(8, GREEN_C, "MAIN_10 - UB_10 : IN 3.3[V] - OUT 3.3[V]:OK");
			Write_String(9, GREEN_C, "MAIN_12 - UB_12 : IN 3.3[V] - OUT 3.3[V]:OK");
			Write_String(10, GREEN_C, "MAIN_13 - UB_13 : IN 3.3[V] - OUT 3.3[V]:OK");
			Write_String(11, GREEN_C, "MAIN_14 - UB_14 : IN 3.3[V] - OUT 3.3[V]:OK");
			Write_String(12, GREEN_C, "MAIN_15 - UB_15 : IN 3.3[V] - OUT 3.3[V]:OK");*/

  while (1)
  {
		/*test_start();
		if(status.display == 0)
		{
			banner();
			Write_String(0, GREEN_C, "MAIN_1-UB_1 : IN 3.3[V] / O : 5 [R] / S : 5 [MR]-> OK");
			Write_String(1, GREEN_C, "MAIN_2-UB_2 : IN 3.3[V] / O : 5 [R] / S : 5 [MR]-> OK");
			Write_String(2, GREEN_C, "MAIN_4-UB_4 : IN 3.3[V] / O : 5 [R] / S : 5 [MR-> OK");
			Write_String(3, GREEN_C, "MAIN_5-UB_5 : IN 3.3[V] / O : 5 [R] / S : 5 [MR]-> OK");
			Write_String(4, GREEN_C, "MAIN_6-UB_6 : IN 3.3[V] / O : 5 [R] / S : 5 [MR]-> OK");
			Write_String(5, GREEN_C, "MAIN_7-UB_7 : IN 3.3[V] / O : 5 [R] / S : 5 [R]-> OK");
			Write_String(6, GREEN_C, "MAIN_8-UB_8 : IN 3.3[V] / O : 5 [R] / S : 5 [R]-> OK");
			Write_String(7, GREEN_C, "MAIN_9-UB_9 : IN 3.3[V] / O : 5 [R] / S : 5 [R]-> OK");
			Write_String(8, GREEN_C, "MAIN_10-UB_10 : IN 3.3[V] / O : 5 [R] / S : 5 [R]-> OK");
			Write_String(9, GREEN_C, "MAIN_12-UB_12 : IN 3.3[V] / O : 5 [R] / S : 5 [MR]-> OK");
			Write_String(10, GREEN_C, "MAIN_13-UB_13 : IN 3.3[V] / O : 5 [R] / S : 5 [MR]-> OK");	
			Write_String(11, GREEN_C, "MAIN_14-UB_14 : IN 3.3[V] / O : 5 [R] / S : 5 [MR]-> OK");
			Write_String(12, GREEN_C, "MAIN_15-UB_15 : IN 3.3[V] / O : 5 [R] / S : 5 [MR]-> OK");
			status.display = 1;
		}*/
		
		FET_20R_ON;
		WriteBit_74HC (U1, HC74_PIN2, HIGH);
		WriteBit_74HC (U12, HC74_PIN1, LOW);
		HAL_ADC_Start(&hadc1);
		adc_value = HAL_ADC_GetValue(&hadc1);
		
  }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_1) 
	{
		VAN_ON;
		status.test = 1;
	}
}

/******** gham hien thi banner *****/
void banner(void)
{
	Nextion_SetColor(MODEL, YELLOW );
	Nextion_Send_String(MODEL, "SM-S928U UB CTC");
	
	Nextion_SetColor(F_W, YELLOW );
	Nextion_Send_String(F_W, "REV0.6");
	
	Nextion_SetColor("t9", YELLOW );
	Nextion_Send_String("t9", "02:18:24");
	
}

/******** ham khoi dong bat dau test *****/
void test_start(void)
{
	if(status.test == 1)
	{
		if(HAL_GetTick() - time > 100)
		{
			count.time ++;
			time = HAL_GetTick();
		}
		if(count.time == 15) status.test = 2;
	}
}

/********* ham bat dau test *****/
void test_testing(void)
{
	//if(status.test == 2)
	
	
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim2.Init.Prescaler = 100-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffffff-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 13, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB4 PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC7 PC8 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 12, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

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
