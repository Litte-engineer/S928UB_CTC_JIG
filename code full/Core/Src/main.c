
#include "main.h"
#include "Nextion.h"
#include "delay.h"
#include "sofware_i2c.h"
#include "MCP23017.h"
#include "Spen_74hc.h"
#include "pinmap.h"
#include "stdio.h"

ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

char* lcd_data1[60] = 
{
	"MAIN_1-UB_1 : IN 3.3[V] / O : 5 [R] /S : 5[R]-> OK",
	"MAIN_2-UB_2 : IN 3.3[V] / O : 3 [R] /S : 3[R]-> OK",
	"MAIN_4-UB_4 : IN 3.3[V] / O : 3 [R] /S : 3[R]-> OK",
	"MAIN_5-UB_5 : IN 3.3[V] / O : 3 [R] /S : 3[R]-> OK",
	"MAIN_6-UB_6 : IN 3.3[V] / O : 7 [R] /S : 7[R]-> OK",
	"MAIN_7-UB_7 : IN 3.3[V] / O : 6 [R] /S : 6[R]-> OK",
	"MAIN_8-UB_8 : IN 3.3[V] / O : 5 [R] /S : 5[R]-> OK",
	"MAIN_9-UB_9 : IN 3.3[V] / O : 9 [R] /S : 9[R]-> OK",
	"MAIN_10-UB_10 : IN 3.3[V] / O : 2 [R] /S : 2[R]-> OK",
	"MAIN_11-UB_11 : IN 3.3[V] / O : 2 [R] /S : 2[R]-> OK",
	"MAIN_12-UB_12 : IN 3.3[V] / O : 3 [R] /S : 3[R]-> OK",
	"MAIN_13-UB_13 : IN 3.3[V] / O : 3 [R] /S : 3[R]-> OK",
	"MAIN_14-UB_14 : IN 3.3[V] / O : 3 [R] /S : 3[R]-> OK",
	"MAIN_15-UB_15 : IN 3.3[V] / O : 3 [R] /S : 3[R]-> OK",
	"MAIN_16-UB_16 : IN 3.3[V] / O : 5 [R] /S : 5[R]-> OK",
	"MAIN_17-UB_17 : IN 3.3[V] / O : 7 [R] /S : 7[R]-> OK",
	"MAIN_18-UB_18 : IN 3.3[V] / O : 5 [R] /S : 5[R]-> OK",
	"MAIN_19-UB_19 : IN 3.3[V] / O : 7 [R] /S : 7[R]-> OK",
	"MAIN_20-UB_20 : IN 3.3[V] / O : 5 [R] /S : 5[R]-> OK",
	
	"MAIN_21-UB_1 : IN 3.3[V] / O : 5 [R] /S : 5[R]-> OK",
	"MAIN_22-UB_2 : IN 3.3[V] / O : 3 [R] /S : 3[R]-> OK",
	"MAIN_24-UB_24 : IN 3.3[V] / O : 3 [R] /S : 3[R]-> OK",
	"MAIN_25-UB_25 : IN 3.3[V] / O : 3 [R] /S : 3[R]-> OK",
	"MAIN_26-UB_26 : IN 3.3[V] / O : 7 [R] /S : 7[R]-> OK",
	"MAIN_27-UB_26 : IN 3.3[V] / O : 6 [R] /S : 6[R]-> OK",
	"MAIN_27-UB_27 : IN 3.3[V] / O : 5 [R] /S : 5[R]-> OK",
	"MAIN_28-UB_28 : IN 3.3[V] / O : 9 [R] /S : 9[R]-> OK",
	"MAIN_29-UB_29 : IN 3.3[V] / O : 2 [R] /S : 2[R]-> OK",
	"MAIN_30-UB_30 : IN 3.3[V] / O : 2 [R] /S : 2[R]-> OK",
	"MAIN_31-UB_32 : IN 3.3[V] / O : 3 [R] /S : 3[R]-> OK",
	"MAIN_33-UB_33 : IN 3.3[V] / O : 3 [R] /S : 3[R]-> OK",
	"MAIN_34-UB_34 : IN 3.3[V] / O : 3 [R] /S : 3[R]-> OK",
	"MAIN_35-UB_35 : IN 3.3[V] / O : 3 [R] /S : 3[R]-> OK",
	"MAIN_36-UB_36 : IN 3.3[V] / O : 5 [R] /S : 5[R]-> OK",
	"MAIN_37-UB_37 : IN 3.3[V] / O : 7 [R] /S : 7[R]-> OK",
	"MAIN_38-UB_38 : IN 3.3[V] / O : 5 [R] /S : 5[R]-> OK",
	
	"MAIN_39-UB_39 : IN 3.3[V] / O : 5 [R] /S : 5[R]-> OK",
	"MAIN_40-UB_40 : IN 3.3[V] / O : 3 [R] /S : 3[R]-> OK",
	"MAIN_41-UB_41 : IN 3.3[V] / O : 3 [R] /S : 3[R]-> OK",
	"MAIN_42-UB_42 : IN 3.3[V] / O : 3 [R] /S : 3[R]-> OK",
	"MAIN_43-UB_43 : IN 3.3[V] / O : 7 [R] /S : 7[R]-> OK",
	"MAIN_44-UB_44 : IN 3.3[V] / O : 6 [R] /S : 6[R]-> OK",
	"MAIN_45-UB_45 : IN 3.3[V] / O : 5 [R] /S : 5[R]-> OK",
	"MAIN_46-UB_46 : IN 3.3[V] / O : 9 [R] /S : 9[R]-> OK",
	"MAIN_47-UB_47 : IN 3.3[V] / O : 2 [R] /S : 2[R]-> OK",
	"MAIN_48-UB_48 : IN 3.3[V] / O : 2 [R] /S : 2[R]-> OK",
	"MAIN_49-UB_49 : IN 3.3[V] / O : 3 [R] /S : 3[R]-> OK",
	"MAIN_50-UB_50 : IN 3.3[V] / O : 3 [R] /S : 3[R]-> OK",
	"MAIN_51-UB_51 : IN 3.3[V] / O : 3 [R] /S : 3[R]-> OK",
	"MAIN_52-UB_52 : IN 3.3[V] / O : 3 [R] /S : 3[R]-> OK",
	"MAIN_53-UB_53 : IN 3.3[V] / O : 5 [R] /S : 5[R]-> OK",
	"MAIN_54-UB_54 : IN 3.3[V] / O : 7 [R] /S : 7[R]-> OK",
	"MAIN_55-UB_55 : IN 3.3[V] / O : 5 [R] /S : 5[R]-> OK",
	
	"MAIN_56-UB_56 : IN 3.3[V] / O : 3 [R] /S : 3[R]-> OK",
	"MAIN_57-UB_57 : IN 3.3[V] / O : 3 [R] /S : 3[R]-> OK",
	"MAIN_58-UB_58 : IN 3.3[V] / O : 3 [R] /S : 3[R]-> OK",
	"MAIN_59-UB_59 : IN 3.3[V] / O : 3 [R] /S : 3[R]-> OK",
	"MAIN_60-UB_60 : IN 3.3[V] / O : 5 [R] /S : 5[R]-> OK",
	"MAIN_61-UB_61 : IN 3.3[V] / O : 7 [R] /S : 7[R]-> OK",
	"MAIN_62-UB_62 : IN 3.3[V] / O : 5 [R] /S : 5[R]-> OK",
};

char* lcd_text_color[MAX_CAPCHE];
char* test_result[20];
uint8_t lcd_count_line1;
uint8_t lcd_old_line_mumber;
uint8_t lcd_status_touch;
uint8_t lcd_reset_line_up;
 uint8_t lcd_count_line;

void test_start(void);
void test_testing(void);
void banner(void);
void real_time(void);

uint32_t time;
uint16_t adc_value;
uint32_t r_spec;
char buff_time[50];

typedef struct
{
	uint16_t time;
	
}Count;
Count count;


typedef struct
{
	uint8_t test;
	uint8_t display;
	uint8_t banner;
	uint8_t sensor;
}Status;
Status status;

typedef struct
{
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
	uint8_t status;
	uint32_t time;
	uint8_t count;
	uint8_t old_count;
}TIME;
TIME Time;

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
	
	delay_init();
	I2C_Init();	
	HAL_Delay(1000);
	LCD_ON;
	HAL_Delay(1000);
	VAN_OFF;
	
	BUZZER_ON;
	HAL_Delay(50);
	BUZZER_OFF;
	HAL_Delay(50);
	BUZZER_ON;
	HAL_Delay(50);
	BUZZER_OFF;
	
	//MCP23017_Init(MCP23017_DEVICE_1);
	//MCP23017_Init(MCP23017_DEVICE_2);
	
	SetOutput_Gpio_A(MCP23017_DEVICE_1);
	SetOutput_Gpio_B(MCP23017_DEVICE_1);
	SetOutput_Gpio_A(MCP23017_DEVICE_2);
	SetOutput_Gpio_B(MCP23017_DEVICE_2);

  while (1)
  {
		HAL_ADC_Start(&hadc1);
		adc_value = HAL_ADC_GetValue(&hadc1);
		status.sensor = SENSOR_READ ;
		real_time();
		test_start();
		banner();

  }
  /* USER CODE END 3 */
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_1) 
	{
		VAN_ON;
		status.test = 1;
		Time.hour = 0;
		Time.min = 0;
		Time.sec = 0;
		Time.status = OFF;
	}
	
	if(GPIO_Pin == GPIO_PIN_15)
	{
		VAN_OFF;
		status.test = 0;
		Time.status = PAUSE;
		HAL_TIM_Base_Stop_IT(&htim3);
	}
}

/******* ham bat dau test *******/
void test_start(void)
{
	if(status.test == 1)
	{
		if(status.sensor == SENSOR_ON) 
		{
			HAL_TIM_Base_Start_IT(&htim3);
			Time.status = ON;
			status.test = 2;
		}
	}
	if(status.test == 2)
	{
		if(lcd_count_line1 < 59)
		{
			if(lcd_count_line1 < 13)
			{
				Write_String(lcd_count_line1, GREEN_C, lcd_data1[lcd_count_line1]);
				sprintf(buff_time,"%02d:%02d:%02d", Time.hour, Time.min, Time.sec);
				Nextion_Send_String("t9", buff_time);
				lcd_count_line1 ++;
				HAL_Delay(100);
			}
			else
			{
				for(uint8_t i = 0; i < 12; i ++)
				{
					Write_String(i, GREEN_C, lcd_data1[lcd_count_line1 - 12 + i]);
					sprintf(buff_time,"%02d:%02d:%02d", Time.hour, Time.min, Time.sec);
					Nextion_Send_String("t9", buff_time);
					lcd_count_line1  ++;
					HAL_Delay(100);
				}	
			}
		}
		else 
		{
			lcd_count_line1 = 0;
			status.test = 3;
		}
		
	}
	if(status.test == 3)
	{
		if(status.sensor == SENSOR_ON) 
		{
			lcd_count_line1 = 0;	
			status.test = 2;				
		}		
	}
}

/****** hien banner *****/
void banner(void)
{
	if(status.banner == 0)
	{
		Nextion_SetColor(MODEL, YELLOW );
		Nextion_Send_String(MODEL, "SM-S928U UB CTC");	
		Nextion_SetColor(F_W, YELLOW );
		Nextion_Send_String(F_W, "REV0.6");
		Nextion_Send_String("t9", "00:00:00");
		status.banner = 1;
	}
}
/***** ham dem thoi gian test thuc te *****/
void real_time(void)
{
		if(HAL_GetTick() - Time.time > 1000)
		{
			Time.count ++;
			Time.time = HAL_GetTick();
		}
		if(Time.old_count != Time.count)
		{
			sprintf(buff_time,"%02d:%02d:%02d", Time.hour, Time.min, Time.sec);
			Nextion_Send_String("t9", buff_time);		
			Time.old_count = Time.count;
		}
}


/********* ngat timer 3 ******/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
		
    Time.sec ++;
		if(Time.sec > 59)
		{
			
			Time.min ++;
			if(Time.min > 59)
			{
				Time.hour++;
				Time.min = 0;
			}
			Time.sec = 0;
		}	
  }
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 9999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB1 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB4 PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC7 PC8 PC9 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_11;
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

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 14, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
