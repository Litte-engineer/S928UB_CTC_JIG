/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  *
  *
  *
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define FET_ON 1
#define FET_OFF 0


uint8_t gUart1_Rx_Buff[10];
uint8_t gUart2_Rx_Buff[1];
uint8_t gkey_count;
uint8_t gmode_count;

volatile pba_bool tft_state = pba_false;
volatile pba_bool touch_state = pba_false;
volatile pba_bool mode_state = pba_false;
volatile pba_bool Rst_Flag = pba_false;
volatile pba_bool Start_Flag = pba_false;
volatile pba_bool Result_Flag = pba_false, Test_Result_Flag = pba_false;;
volatile pba_bool Push_Flag = pba_false;
volatile pba_bool Debug_Flag = pba_false;
volatile pba_bool In_ostate = pba_true, Out_ostate = pba_true, In_sstate = pba_true, Out_sstate = pba_true;
volatile pba_bool page3_state = pba_false, page4_state = pba_false;

uint8_t g_buff[200];
uint8_t g_touch_buff[9];
uint8_t g_rbuf[4] = {0,};
TS_StateTypeDef TS;
uint32_t g_ok_count, g_ng_count, g_total_count,g_clean, in_ok_count, in_ng_count, in_sum_count, out_ok_count, out_ng_count, out_sum_count, total_ok_count, total_ng_count, total_sum_count;
uint8_t ok_count = 0, open_count = 0, short_count = 0;
uint32_t g_type = 0, g_mark = 0, g_mode = 0, g_error = 0, g_test = 0, g_lock_on = 0, g_key_on = 0;
uint32_t g_tick_timer1 = 0, g_tick_timer2 = 0;
uint32_t temp = 0, giay = 0, phut =0, gio =0 ;
char* g_model_name = "SM-S928U_UB CTC";
char* g_fw_ver = "V0.0.2";

uint8_t Vaccum_Push = 0;
uint8_t count1 = 0;
uint8_t count2 = 0;
uint8_t count3 = 0;

static float OS_Function(OS_STATE value);
static float R_Function(void);
static void Fet_spec(uint8_t fet, uint8_t status);
int flag=0;

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_DMA(&huart1, gUart1_Rx_Buff, 10);
  HAL_UART_Receive_DMA(&huart2, gUart2_Rx_Buff, 1);

 setbuf(stdout, NULL);

  system_information(); //eep read

#if 0
  //º£Æ®³² ¿äĂ»À¸·Î Ăß°¡µÈ ·çÆ¾ °Ë»ç±â 1 to 3
  if(g_test == DEBUG_ON)
  {
	  //routine-1
	  BSP_TS_Init(0,0);

	  HAL_GPIO_WritePin(LCD_POWER_GPIO_Port, LCD_POWER_Pin, GPIO_PIN_SET); //LCD Power ON
	  HAL_Delay(500);

	  TFT_Serial((uint8_t*)"bkcmd=0");
	  TFT_Serial((uint8_t*)"page 3");

	  sprintf((char*)g_buff, "t0.txt=\"%s\"", g_model_name);
	  TFT_Serial(g_buff);

	  sprintf((char*)g_buff, "t1.txt=\"%s\"", g_fw_ver);
	  TFT_Serial(g_buff);

	  for(;;)
	  {
		  if(BSP_TS_GetState(&TS) == TS_OK)
		  {
			  Beep(1,50);

			  if(g_touch_buff[5] == 'B')
			  {
				  //OK Touch
				  page3_state = pba_true;
			  }
			  else if(g_touch_buff[5] == 'C')
			  {
				  //NG Touch
			  }

			  HAL_UART_Receive_DMA(&huart1, gUart1_Rx_Buff, 9);
		  }

		  if(page3_state == pba_true) break;
	  }

	  TFT_Serial((uint8_t*)"page 4");

	  if(g_type == IN) TFT_Serial((uint8_t*)"t0.txt=\"IN SAMPLE TEST MODE\"");
	  else if(g_type == OUT) TFT_Serial((uint8_t*)"t0.txt=\"OUT SAMPLE TEST MODE\"");
	  else if(g_type == ALL) TFT_Serial((uint8_t*)"t0.txt=\"BOTH SAMPLE TEST MODE\"");

	  mcp23017_init(SLAVE0);
	  mcp23017_init(SLAVE1);
	  mcp23017_init(SLAVE2);

	  Vaccum_Push_Set();

	  for(;;)
	  {
		  Key_input();
		  Start_Task();
		  Rst_Task();

		  if(Test_mode_Result() == pba_true)
		  {
			  page4_state = pba_true;
			  break;
		  }
	  }

	  HAL_GPIO_WritePin(LCD_POWER_GPIO_Port, LCD_POWER_Pin, GPIO_PIN_RESET); //LCD Power OFF
  }
  else if(g_test == DEBUG_OFF)
  {
	  page4_state = pba_true;
  }
#endif

  {
	  page4_state = pba_true;
	  //System Init : ½ºÄÚÇÁ ³» ¼öÁ¤ ±ƯÁö
	  BSP_LCD_Init();
	  LCD_LOG_Init();
	  BSP_TS_Init(0,0);

	  LCD_DbgLog("LCD&Touch-screen Init OK\n");

	  Banner_Display();
	  Count_Display();

	  LCD_UsrLog("*****************************************\n");
	  LCD_UsrLog("*                                       *\n");
	  LCD_UsrLog("*                Copyright              *\n");
	  LCD_UsrLog("*                                       *\n");
	  LCD_UsrLog("*****************************************\n");
	  LCD_UsrLog("1. H/W Version : FPCB Cell Mainboard V2\n");
	  LCD_UsrLog("2. Boot Version : USB_Bootloader_V1\n");
	  LCD_UsrLog("3. S/W Released by Synopex IT Laboratory\n");
	  LCD_UsrLog("=========================================\n");

	  mcp23017_init(SLAVE0);
	  mcp23017_init(SLAVE1);
	  mcp23017_init(SLAVE2);

	  DFF_Floating_Init();

	  LCD_DbgLog("System Initialized OK.\n");
	  Beep(2,50);
  }

  if(g_test == DEBUG_ON)
   {
 	  TFT_Serial((uint8_t*)"page 3");

 	  sprintf((char*)g_buff, "t0.txt=\"%s\"", g_model_name);
 	  TFT_Serial(g_buff);

 	  sprintf((char*)g_buff, "t1.txt=\"%s\"", g_fw_ver);
 	  TFT_Serial(g_buff);

 	  for(;;)
 	  {
 		  if(BSP_TS_GetState(&TS) == TS_OK)
 		  {
 			  Beep(1,50);

 			  if(g_touch_buff[5] == 'B')
 			  {
 				  //OK Touch
 				  page3_state = pba_true;
 			  }
 			  else if(g_touch_buff[5] == 'C')
 			  {
 				  //NG Touch
 			  }

 			  HAL_UART_Receive_DMA(&huart1, gUart1_Rx_Buff, 9);
 		  }

 		  if(page3_state == pba_true)
 		  {
 			  TFT_Serial((uint8_t*)"page 4");
 			  if(g_type == IN) TFT_Serial((uint8_t*)"t0.txt=\"IN SAMPLE TEST MODE\"");
 //			  else if(g_type == OUT) TFT_Serial((uint8_t*)"t0.txt=\"OUT SAMPLE TEST MODE\"");
 //			  else if(g_type == ALL) TFT_Serial((uint8_t*)"t0.txt=\"BOTH SAMPLE TEST MODE\"");
 			  break;
 		  }
 	  }
   }

  Vaccum_Push_Set();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Touch_Task(); //LCD Touch
	  Mode_Task(); //Mode key
	  Key_input(); //Start Key
	  Rst_Task(); //Rst Key
	  Start_Task(); //Inspection routine
	  Result_Task(); //Result & Marking
//	  Test_mode_Result(); //TEST mode result
	  //Error_Stop();
	  Auto_Clean();
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
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
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
	  Error_Handler();
  }
  /* USER CODE END ADC1_Init 2 */

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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 13, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 14, 0);
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CY9_Pin|CY10_Pin|CY8_Pin|CY7_Pin 
                          |SW_SDA_Pin|SW_SCL_Pin|LCD_POWER_Pin|BUZZER_Pin 
                          |CY4_Pin|CY2_Pin|CY3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|CY6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RLED_OK_Pin|RLED_NG_Pin|FET3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, NG_LED_Pin|FET2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, FET4_Pin|OK_LED_Pin|FET5_Pin|FET6_Pin 
                          |FET1_Pin|LLED_OK_Pin|LLED_NG_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CY1_GPIO_Port, CY1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CY5_GPIO_Port, CY5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CY9_Pin CY10_Pin CY8_Pin CY7_Pin 
                           NG_LED_Pin SW_SDA_Pin SW_SCL_Pin FET2_Pin 
                           LCD_POWER_Pin BUZZER_Pin CY4_Pin CY2_Pin 
                           CY3_Pin */
  GPIO_InitStruct.Pin = CY9_Pin|CY10_Pin|CY8_Pin|CY7_Pin 
                          |NG_LED_Pin|SW_SDA_Pin|SW_SCL_Pin|FET2_Pin 
                          |LCD_POWER_Pin|BUZZER_Pin|CY4_Pin|CY2_Pin 
                          |CY3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin RLED_OK_Pin RLED_NG_Pin FET3_Pin 
                           CY6_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|RLED_OK_Pin|RLED_NG_Pin|FET3_Pin 
                          |CY6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LEFT_KEY_Pin MODE_KEY_Pin SKIP_KEY_Pin RIGHT_KEY_Pin */
  GPIO_InitStruct.Pin = LEFT_KEY_Pin|MODE_KEY_Pin|SKIP_KEY_Pin|RIGHT_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : FET4_Pin OK_LED_Pin FET5_Pin FET6_Pin 
                           FET1_Pin CY5_Pin LLED_OK_Pin LLED_NG_Pin */
  GPIO_InitStruct.Pin = FET4_Pin|OK_LED_Pin|FET5_Pin|FET6_Pin 
                          |FET1_Pin|CY5_Pin|LLED_OK_Pin|LLED_NG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RESET_KEY_Pin */
  GPIO_InitStruct.Pin = RESET_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(RESET_KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CY1_Pin */
  GPIO_InitStruct.Pin = CY1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CY1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
extern int __io_putchar(int ch);

int _write(int file, uint8_t *ptr, int len)
{
	int DataIdx;

	if(*ptr == '\n')
	{
		uint8_t ch_temp = '\r';
		HAL_UART_Transmit(&huart2, &ch_temp, 1, 0xFFFF);
	}

	HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, 0xFFFF);

	if(page4_state == pba_true)
	{
		for (DataIdx = 0; DataIdx < len; DataIdx++)
		{
			__io_putchar(*ptr++);
		}
	}

	return len;
}

void Debug_serial(char *buff)
{
	uint8_t num = strlen(buff);

	HAL_UART_Transmit(&huart2, (uint8_t*)buff, num, 0xFFFFFFFF);
	//HAL_UART_Transmit(&huart2, (uint8_t*)"\r", 1, 0xFFFFFFFF);
	//HAL_UART_Transmit(&huart2, (uint8_t*)"\n", 1, 0xFFFFFFFF);
}

void TFT_Serial(uint8_t* string)
{
	char END_FILE[3] = {0xFF, 0xFF, 0xFF};

	uint8_t size = strlen((char*)string);

	strcpy((char*)g_buff, (char*)string);
	strcat((char*)g_buff, END_FILE);

	HAL_UART_Transmit(&huart1, g_buff, size+3, 0xFFFF);

	HAL_Delay(5); //ÇÊ¼ö
	//if(g_mode == 0) HAL_Delay(5);
	//else HAL_Delay(2);
}

void Beep(uint8_t count, uint32_t delay)
{
	for(int i=0; i<count; i++)
	{
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
		HAL_Delay(delay);
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
		HAL_Delay(delay);
	}
}

void Result_Display(pba_bool result_in, pba_bool result_out)
{
	BSP_LCD_Clear(BLACK);

	if(result_in == pba_true)
	{
		sprintf((char*)g_buff,"xstr 20,61,300,80,5,GREEN,BLACK,0,0,1,\"IN :OK\"");
		TFT_Serial(g_buff);

		sprintf((char*)g_buff,"xstr 380,70,95,50,2,GREEN,BLACK,1,1,1,\"OK\"");
		TFT_Serial(g_buff);
	}
	else if(result_in == pba_false)
	{
		sprintf((char*)g_buff,"xstr 20,61,300,80,5,RED,BLACK,0,0,1,\"IN :NG\"");
		TFT_Serial(g_buff);

		sprintf((char*)g_buff,"xstr 380,70,95,50,2,RED,BLACK,1,1,1,\"NG\"");
		TFT_Serial(g_buff);
	}
	else
	{
		sprintf((char*)g_buff,"xstr 20,61,300,80,5,WHITE,BLACK,0,0,1,\"IN :-\"");
		TFT_Serial(g_buff);

		sprintf((char*)g_buff,"xstr 380,70,95,50,2,WHITE,BLACK,1,1,1,\"-\"");
		TFT_Serial(g_buff);
	}

	if(result_out == pba_true)
	{
		sprintf((char*)g_buff,"xstr 20,161,300,80,5,GREEN,BLACK,0,0,1,\"OUT:OK\"");
		TFT_Serial(g_buff);

		sprintf((char*)g_buff,"xstr 380,134,95,50,2,GREEN,BLACK,1,1,1,\"OK\"");
		TFT_Serial(g_buff);
	}
	else if(result_out == pba_false)
	{
		sprintf((char*)g_buff,"xstr 20,161,300,80,5,RED,BLACK,0,0,1,\"OUT:NG\"");
		TFT_Serial(g_buff);

		sprintf((char*)g_buff,"xstr 380,134,95,50,2,RED,BLACK,1,1,1,\"NG\"");
		TFT_Serial(g_buff);
	}
	else
	{
		sprintf((char*)g_buff,"xstr 20,161,300,80,5,WHITE,BLACK,0,0,1,\"OUT:-\"");
		TFT_Serial(g_buff);

		sprintf((char*)g_buff,"xstr 380,134,95,50,2,WHITE,BLACK,1,1,1,\"-\"");
		TFT_Serial(g_buff);
	}
}

void system_information(void)
{
	uint8_t rbuf[4] = {0,};

	eep_256_burst_read(TYPE_ADDRESS, rbuf, 4);
	g_type = ((uint32_t)rbuf[3] << 24) + ((uint32_t)rbuf[2] << 16) + ((uint32_t)rbuf[1] << 8) + (uint32_t)rbuf[0];
	if(g_type != IN && g_type != OUT && g_type != ALL)
	{
		memset(rbuf, 0, 4);
		eep_256_burst_write(TYPE_ADDRESS, rbuf, 4); //0 Clear, default IN+OUT

		eep_256_burst_read(TYPE_ADDRESS, rbuf, 4);
		g_type = ((uint32_t)rbuf[3] << 24) + ((uint32_t)rbuf[2] << 16) + ((uint32_t)rbuf[1] << 8) + (uint32_t)rbuf[0];
	}

	eep_256_burst_read(MARK_ADDRESS, rbuf, 4);
	g_mark = ((uint32_t)rbuf[3] << 24) + ((uint32_t)rbuf[2] << 16) + ((uint32_t)rbuf[1] << 8) + (uint32_t)rbuf[0];
	if(g_mark != MARK_ON && g_mark !=  MARK_OFF)
	{
		memset(rbuf, 0, 4);
		eep_256_burst_write(MARK_ADDRESS, rbuf, 4); //0 Clear, default MARK_ON

		eep_256_burst_read(MARK_ADDRESS, rbuf, 4);
		g_mark = ((uint32_t)rbuf[3] << 24) + ((uint32_t)rbuf[2] << 16) + ((uint32_t)rbuf[1] << 8) + (uint32_t)rbuf[0];
	}

	eep_256_burst_read(DEBUG_ADDRESS, rbuf, 4);
	g_mode = ((uint32_t)rbuf[3] << 24) + ((uint32_t)rbuf[2] << 16) + ((uint32_t)rbuf[1] << 8) + (uint32_t)rbuf[0];
	if(g_mode != DEBUG_ON && g_mode != DEBUG_OFF)
	{
		memset(rbuf, 0, 4);
		eep_256_burst_write(DEBUG_ADDRESS, rbuf, 4); //0 Clear, default DEBUG ON

		eep_256_burst_read(DEBUG_ADDRESS, rbuf, 4);
		g_mode = ((uint32_t)rbuf[3] << 24) + ((uint32_t)rbuf[2] << 16) + ((uint32_t)rbuf[1] << 8) + (uint32_t)rbuf[0];
	}

	eep_256_burst_read(ERROR_STOP_ADDRESS, rbuf, 4);
	g_error = ((uint32_t)rbuf[3] << 24) + ((uint32_t)rbuf[2] << 16) + ((uint32_t)rbuf[1] << 8) + (uint32_t)rbuf[0];
	if(g_error != DEBUG_ON && g_error != DEBUG_OFF)
	{
		memset(rbuf, 0, 4);
		eep_256_burst_write(ERROR_STOP_ADDRESS, rbuf, 4); //0 Clear, default ERROR STOP ON

		eep_256_burst_read(ERROR_STOP_ADDRESS, rbuf, 4);
		g_error = ((uint32_t)rbuf[3] << 24) + ((uint32_t)rbuf[2] << 16) + ((uint32_t)rbuf[1] << 8) + (uint32_t)rbuf[0];
	}

	eep_256_burst_read(TEST_MODE_ADDRESS, rbuf, 4);
	g_test = ((uint32_t)rbuf[3] << 24) + ((uint32_t)rbuf[2] << 16) + ((uint32_t)rbuf[1] << 8) + (uint32_t)rbuf[0];
	if(g_test != DEBUG_ON && g_test != DEBUG_OFF)
	{
		memset(rbuf, 0, 4);
		eep_256_burst_write(TEST_MODE_ADDRESS, rbuf, 4); //0 Clear, default TEST MODE OFF

		eep_256_burst_read(TEST_MODE_ADDRESS, rbuf, 4);
		g_test = ((uint32_t)rbuf[3] << 24) + ((uint32_t)rbuf[2] << 16) + ((uint32_t)rbuf[1] << 8) + (uint32_t)rbuf[0];
	}

	eep_256_burst_read(LOCK_ON_ADDRESS, rbuf, 4);
	g_lock_on = ((uint32_t)rbuf[3] << 24) + ((uint32_t)rbuf[2] << 16) + ((uint32_t)rbuf[1] << 8) + (uint32_t)rbuf[0];
	if(g_lock_on != LOCK_ON && g_lock_on != LOCK_OFF)
	{
		memset(rbuf, 0, 4);
		eep_256_burst_write(LOCK_ON_ADDRESS, rbuf, 4); //0 Clear, default LOCK_OFF

		eep_256_burst_read(LOCK_ON_ADDRESS, rbuf, 4);
		g_lock_on = ((uint32_t)rbuf[3] << 24) + ((uint32_t)rbuf[2] << 16) + ((uint32_t)rbuf[1] << 8) + (uint32_t)rbuf[0];
	}

	eep_256_burst_read(KEY_INPUT_ADDRESS, rbuf, 4);
	g_key_on = ((uint32_t)rbuf[3] << 24) + ((uint32_t)rbuf[2] << 16) + ((uint32_t)rbuf[1] << 8) + (uint32_t)rbuf[0];
	if(g_key_on != KEY_ON && g_key_on != KEY_OFF)
	{
		memset(rbuf, 0, 4);
		eep_256_burst_write(KEY_INPUT_ADDRESS, rbuf, 4); //0 Clear, default LOCK_OFF

		eep_256_burst_read(KEY_INPUT_ADDRESS, rbuf, 4);
		g_key_on = ((uint32_t)rbuf[3] << 24) + ((uint32_t)rbuf[2] << 16) + ((uint32_t)rbuf[1] << 8) + (uint32_t)rbuf[0];
	}
}

void Banner_Display(void)
{
	system_information();

/*	if(g_type == IN) TFT_Serial((uint8_t*)"t9.txt=\"IN\"");
	else if(g_type == OUT) TFT_Serial((uint8_t*)"t9.txt=\"OUT\"");
	else if(g_type == ALL) TFT_Serial((uint8_t*)"t9.txt=\"IN+OUT\"");*/

	if(g_mode == DEBUG_ON) TFT_Serial((uint8_t*)"t10.txt=\"Debug ON\"");
	else if(g_mode == DEBUG_OFF) TFT_Serial((uint8_t*)"t10.txt=\"Debug OFF\"");

	sprintf((char*)g_buff, "t7.txt=\"%s\"", g_model_name);
	TFT_Serial(g_buff);

	sprintf((char*)g_buff, "t8.txt=\"%s\"", g_fw_ver);
	TFT_Serial(g_buff);
}

void Count_Display(void)
{
	uint8_t rbuf[4] = {0,};

	eep_256_burst_read(IN_OK_ADDRESS, rbuf, 4);
	in_ok_count = ((uint32_t)rbuf[3] << 24) + ((uint32_t)rbuf[2] << 16) + ((uint32_t)rbuf[1] << 8) + (int32_t)rbuf[0];
	sprintf((char*)g_buff, "t16.txt=\"%5ld\"", in_ok_count);
	TFT_Serial(g_buff);

	eep_256_burst_read(IN_NG_ADDRESS, rbuf, 4);
	in_ng_count = ((uint32_t)rbuf[3] << 24) + ((uint32_t)rbuf[2] << 16) + ((uint32_t)rbuf[1] << 8) + (int32_t)rbuf[0];
	sprintf((char*)g_buff, "t24.txt=\"%5ld\"", in_ng_count);
	TFT_Serial(g_buff);

	in_sum_count = in_ok_count + in_ng_count;
	sprintf((char*)g_buff, "t20.txt=\"%5ld\"", in_sum_count);
	TFT_Serial(g_buff);

	eep_256_burst_read(OUT_OK_ADDRESS, rbuf, 4);
	out_ok_count = ((uint32_t)rbuf[3] << 24) + ((uint32_t)rbuf[2] << 16) + ((uint32_t)rbuf[1] << 8) + (int32_t)rbuf[0];
	sprintf((char*)g_buff, "t17.txt=\"%5ld\"", out_ok_count);
	TFT_Serial(g_buff);

	eep_256_burst_read(OUT_NG_ADDRESS, rbuf, 4);
	out_ng_count = ((uint32_t)rbuf[3] << 24) + ((uint32_t)rbuf[2] << 16) + ((uint32_t)rbuf[1] << 8) + (int32_t)rbuf[0];
	sprintf((char*)g_buff, "t26.txt=\"%5ld\"", out_ng_count);
	TFT_Serial(g_buff);

	out_sum_count = out_ok_count + out_ng_count;
	sprintf((char*)g_buff, "t25.txt=\"%5ld\"", out_sum_count);
	TFT_Serial(g_buff);

	total_ok_count = in_ok_count + out_ok_count;
	sprintf((char*)g_buff, "t18.txt=\"%5ld\"", total_ok_count);
	TFT_Serial(g_buff);

	total_ng_count = in_ng_count + out_ng_count;
	sprintf((char*)g_buff, "t28.txt=\"%5ld\"", total_ng_count);
	TFT_Serial(g_buff);

	total_sum_count = in_ok_count + in_ng_count + out_ok_count + out_ng_count;
	sprintf((char*)g_buff, "t29.txt=\"%5ld\"", total_sum_count);
	TFT_Serial(g_buff);

	eep_256_burst_read(CLEAN_ADDRESS, rbuf, 4);
	g_clean = ((uint32_t)rbuf[3] << 24) + ((uint32_t)rbuf[2] << 16) + ((uint32_t)rbuf[1] << 8) + (int32_t)rbuf[0];
	sprintf((char*)g_buff, "t21.txt=\"%5ld\"", g_clean);
	TFT_Serial(g_buff);
}

void Touch_Task(void)
{
	if(BSP_TS_GetState(&TS) == TS_OK)
	{
		if(g_touch_buff[0] == 0x70) //in mode
		{
			uint32_t wdata = 0;

			Beep(1,10);

			if(g_touch_buff[5] == '1')
			{
				//OK Clear
				memset(g_rbuf, 0, 4);
				eep_256_burst_write(IN_OK_ADDRESS, g_rbuf, 4);
				memset(g_rbuf, 0, 4);
				eep_256_burst_write(IN_NG_ADDRESS, g_rbuf, 4);
			}
			else if(g_touch_buff[5] == '2')
			{
				//NG Clear
				memset(g_rbuf, 0, 4);
				eep_256_burst_write(OUT_OK_ADDRESS, g_rbuf, 4);
				memset(g_rbuf, 0, 4);
				eep_256_burst_write(OUT_NG_ADDRESS, g_rbuf, 4);
			}
			else if(g_touch_buff[5] == '3')
			{
				//Total Clear
				memset(g_rbuf, 0, 4);
				eep_256_burst_write(TOTAL_ADDRESS, g_rbuf, 4);
			}
			else if(g_touch_buff[5] == 'Y')
			{
				//Total Clear
				memset(g_rbuf, 0, 4);
				eep_256_burst_write(CLEAN_ADDRESS, g_rbuf, 4);
			}
			else if(g_touch_buff[5] == '4')
			{
				//TYPE : IN
				wdata = IN;

				g_rbuf[0] = (uint8_t)(wdata & 0x000000FF);
				g_rbuf[1] = (uint8_t)((wdata & 0x0000FF00) >> 8);
				g_rbuf[2] = (uint8_t)((wdata & 0x00FF0000) >> 16);
				g_rbuf[3] = (uint8_t)((wdata & 0xFF000000) >> 24);

				eep_256_burst_write(TYPE_ADDRESS, g_rbuf, 4);

				TFT_Serial((uint8_t*)"t4.txt=\"IN\"");
			}
			else if(g_touch_buff[5] == '5')
			{
				//TYPE : OUT
				wdata = OUT;

				g_rbuf[0] = (uint8_t)(wdata & 0x000000FF);
				g_rbuf[1] = (uint8_t)((wdata & 0x0000FF00) >> 8);
				g_rbuf[2] = (uint8_t)((wdata & 0x00FF0000) >> 16);
				g_rbuf[3] = (uint8_t)((wdata & 0xFF000000) >> 24);

				eep_256_burst_write(TYPE_ADDRESS, g_rbuf, 4);

				TFT_Serial((uint8_t*)"t4.txt=\"OUT\"");
			}
			else if(g_touch_buff[5] == '6')
			{
				//TYPE : IN+OUT
				wdata = ALL;

				g_rbuf[0] = (uint8_t)(wdata & 0x000000FF);
				g_rbuf[1] = (uint8_t)((wdata & 0x0000FF00) >> 8);
				g_rbuf[2] = (uint8_t)((wdata & 0x00FF0000) >> 16);
				g_rbuf[3] = (uint8_t)((wdata & 0xFF000000) >> 24);

				eep_256_burst_write(TYPE_ADDRESS, g_rbuf, 4);

				TFT_Serial((uint8_t*)"t4.txt=\"IN+OUT\"");
			}
			else if(g_touch_buff[5] == '7')
			{
				//MARK ON
				memset(g_rbuf, MARK_ON, 4);

				eep_256_burst_write(MARK_ADDRESS, g_rbuf, 4);

				TFT_Serial((uint8_t*)"t3.txt=\"ON\"");
			}
			else if(g_touch_buff[5] == '8')
			{
				//MARK OFF
				wdata = MARK_OFF;

				g_rbuf[0] = (uint8_t)(wdata & 0x000000FF);
				g_rbuf[1] = (uint8_t)((wdata & 0x0000FF00) >> 8);
				g_rbuf[2] = (uint8_t)((wdata & 0x00FF0000) >> 16);
				g_rbuf[3] = (uint8_t)((wdata & 0xFF000000) >> 24);

				eep_256_burst_write(MARK_ADDRESS, g_rbuf, 4);

				TFT_Serial((uint8_t*)"t3.txt=\"OFF\"");
			}
			else if(g_touch_buff[5] == '9')
			{
				//Debug : ON
				wdata = DEBUG_ON;

				g_rbuf[0] = (uint8_t)(wdata & 0x000000FF);
				g_rbuf[1] = (uint8_t)((wdata & 0x0000FF00) >> 8);
				g_rbuf[2] = (uint8_t)((wdata & 0x00FF0000) >> 16);
				g_rbuf[3] = (uint8_t)((wdata & 0xFF000000) >> 24);

				eep_256_burst_write(DEBUG_ADDRESS, g_rbuf, 4);

				TFT_Serial((uint8_t*)"t9.txt=\"ON\"");
			}
			else if(g_touch_buff[5] == 'A')
			{
				//Debug : OFF
				wdata = DEBUG_OFF;

				g_rbuf[0] = (uint8_t)(wdata & 0x000000FF);
				g_rbuf[1] = (uint8_t)((wdata & 0x0000FF00) >> 8);
				g_rbuf[2] = (uint8_t)((wdata & 0x00FF0000) >> 16);
				g_rbuf[3] = (uint8_t)((wdata & 0xFF000000) >> 24);

				eep_256_burst_write(DEBUG_ADDRESS, g_rbuf, 4);

				TFT_Serial((uint8_t*)"t9.txt=\"OFF\"");
			}

			else if(g_touch_buff[5] == 'D')
			{
				system_information();

				TFT_Serial((uint8_t*)"page 5");

				if(g_error == DEBUG_ON) TFT_Serial((uint8_t*)"t4.txt=\"ON\"");
				else if(g_error == DEBUG_OFF) TFT_Serial((uint8_t*)"t4.txt=\"OFF\"");

				if(g_test == DEBUG_ON) TFT_Serial((uint8_t*)"t3.txt=\"ON\"");
				else if(g_test == DEBUG_OFF) TFT_Serial((uint8_t*)"t3.txt=\"OFF\"");

				if(g_key_on == KEY_ON) TFT_Serial((uint8_t*)"t7.txt=\"ON\"");
				else if(g_key_on == KEY_OFF) TFT_Serial((uint8_t*)"t7.txt=\"OFF\"");

			}

			else if(g_touch_buff[5] == 'E')
			{
				//Error_Stop : ON
				wdata = DEBUG_ON;

				g_rbuf[0] = (uint8_t)(wdata & 0x000000FF);
				g_rbuf[1] = (uint8_t)((wdata & 0x0000FF00) >> 8);
				g_rbuf[2] = (uint8_t)((wdata & 0x00FF0000) >> 16);
				g_rbuf[3] = (uint8_t)((wdata & 0xFF000000) >> 24);

				eep_256_burst_write(ERROR_STOP_ADDRESS, g_rbuf, 4);

				TFT_Serial((uint8_t*)"t4.txt=\"ON\"");
			}

			else if(g_touch_buff[5] == 'F')
			{
				//Error_Stop : OFF
				wdata = DEBUG_OFF;

				g_rbuf[0] = (uint8_t)(wdata & 0x000000FF);
				g_rbuf[1] = (uint8_t)((wdata & 0x0000FF00) >> 8);
				g_rbuf[2] = (uint8_t)((wdata & 0x00FF0000) >> 16);
				g_rbuf[3] = (uint8_t)((wdata & 0xFF000000) >> 24);

				eep_256_burst_write(ERROR_STOP_ADDRESS, g_rbuf, 4);

				TFT_Serial((uint8_t*)"t4.txt=\"OFF\"");
			}

			else if(g_touch_buff[5] == 'G')
			{
				//TEST MODE : ON
				wdata = DEBUG_ON;

				g_rbuf[0] = (uint8_t)(wdata & 0x000000FF);
				g_rbuf[1] = (uint8_t)((wdata & 0x0000FF00) >> 8);
				g_rbuf[2] = (uint8_t)((wdata & 0x00FF0000) >> 16);
				g_rbuf[3] = (uint8_t)((wdata & 0xFF000000) >> 24);

				eep_256_burst_write(TEST_MODE_ADDRESS, g_rbuf, 4);

				TFT_Serial((uint8_t*)"t3.txt=\"ON\"");
			}

			else if(g_touch_buff[5] == 'H')
			{
				//TEST MODE : OFF
				wdata = DEBUG_OFF;

				g_rbuf[0] = (uint8_t)(wdata & 0x000000FF);
				g_rbuf[1] = (uint8_t)((wdata & 0x0000FF00) >> 8);
				g_rbuf[2] = (uint8_t)((wdata & 0x00FF0000) >> 16);
				g_rbuf[3] = (uint8_t)((wdata & 0xFF000000) >> 24);

				eep_256_burst_write(TEST_MODE_ADDRESS, g_rbuf, 4);

				TFT_Serial((uint8_t*)"t3.txt=\"OFF\"");
			}

			else if(g_touch_buff[5] == 'I')
			{
				system_information();

				TFT_Serial((uint8_t*)"page 2");

				if(g_type == IN) TFT_Serial((uint8_t*)"t4.txt=\"IN\"");
				else if(g_type == OUT) TFT_Serial((uint8_t*)"t4.txt=\"OUT\"");
				else if(g_type == ALL) TFT_Serial((uint8_t*)"t4.txt=\"IN+OUT\"");

				if(g_mark == MARK_ON) TFT_Serial((uint8_t*)"t3.txt=\"MARK ON\"");
				else if(g_mark == MARK_OFF) TFT_Serial((uint8_t*)"t3.txt=\"MARK OFF\"");

				if(g_mode == DEBUG_ON) TFT_Serial((uint8_t*)"t9.txt=\"ON\"");
				else if(g_mode == DEBUG_OFF) TFT_Serial((uint8_t*)"t9.txt=\"OFF\"");
			}

			else if(g_touch_buff[5] == 'M')
			{
				//KEY_INPUT : ON
				wdata = KEY_ON;

				g_rbuf[0] = (uint8_t)(wdata & 0x000000FF);
				g_rbuf[1] = (uint8_t)((wdata & 0x0000FF00) >> 8);
				g_rbuf[2] = (uint8_t)((wdata & 0x00FF0000) >> 16);
				g_rbuf[3] = (uint8_t)((wdata & 0xFF000000) >> 24);

				eep_256_burst_write(KEY_INPUT_ADDRESS, g_rbuf, 4);

				TFT_Serial((uint8_t*)"t7.txt=\"ON\"");
			}

			else if(g_touch_buff[5] == 'N')
			{
				//KEY_INPUT : OFF
				wdata = KEY_OFF;

				g_rbuf[0] = (uint8_t)(wdata & 0x000000FF);
				g_rbuf[1] = (uint8_t)((wdata & 0x0000FF00) >> 8);
				g_rbuf[2] = (uint8_t)((wdata & 0x00FF0000) >> 16);
				g_rbuf[3] = (uint8_t)((wdata & 0xFF000000) >> 24);

				eep_256_burst_write(KEY_INPUT_ADDRESS, g_rbuf, 4);

				TFT_Serial((uint8_t*)"t7.txt=\"OFF\"");
			}

			else if(g_touch_buff[5] == 'Z')
			{
				//Save & exit
				TFT_Serial((uint8_t*)"page 1");
				mode_state = pba_false;
				BSP_LCD_Init();
				Banner_Display();
				Count_Display();
			}
		}
		else
		{
			if(TS.x > LCD_PIXEL_WIDTH/2)
			{
				LCD_LOG_ScrollForward();
			}
			else if(TS.x < LCD_PIXEL_WIDTH/2)
			{
				LCD_LOG_ScrollBack();
			}
		}

		HAL_UART_Receive_DMA(&huart1, gUart1_Rx_Buff, 9);
	}
}


void Mode_Task(void)
{
	if((HAL_GPIO_ReadPin(MODE_KEY_GPIO_Port, MODE_KEY_Pin) == GPIO_PIN_RESET))
	{
		HAL_Delay(40);

		if((HAL_GPIO_ReadPin(MODE_KEY_GPIO_Port, MODE_KEY_Pin) == GPIO_PIN_RESET))
			gmode_count++;
		//flag=1;
		if(gmode_count == 80)
		{
			Beep(1,50);
			gmode_count = 0;
			count1 = 0;
			count2 = 0;
			count3 = 0;
			Vaccum_Push = 0;

			mode_state = pba_true;

			//In mode key input
			TFT_Serial((uint8_t*)"page 2");

			eep_256_burst_read(TYPE_ADDRESS, g_rbuf, 4);
			g_type = ((uint32_t)g_rbuf[3] << 24) + ((uint32_t)g_rbuf[2] << 16) + ((uint32_t)g_rbuf[1] << 8) + (int32_t)g_rbuf[0];

			eep_256_burst_read(MARK_ADDRESS, g_rbuf, 4);
			g_mark = ((uint32_t)g_rbuf[3] << 24) + ((uint32_t)g_rbuf[2] << 16) + ((uint32_t)g_rbuf[1] << 8) + (int32_t)g_rbuf[0];

			eep_256_burst_read(DEBUG_ADDRESS, g_rbuf, 4);
			g_mode = ((uint32_t)g_rbuf[3] << 24) + ((uint32_t)g_rbuf[2] << 16) + ((uint32_t)g_rbuf[1] << 8) + (int32_t)g_rbuf[0];

			if(g_type == IN) TFT_Serial((uint8_t*)"t4.txt=\"IN\"");

			else if(g_type == OUT) TFT_Serial((uint8_t*)"t4.txt=\"OUT\"");
			else if(g_type == ALL) TFT_Serial((uint8_t*)"t4.txt=\"IN+OUT\"");


			if(g_mark == MARK_ON) TFT_Serial((uint8_t*)"t3.txt=\"MARK ON\"");
			else if(g_mark == MARK_OFF) TFT_Serial((uint8_t*)"t3.txt=\"MARK OFF\"");

			if(g_mode == DEBUG_ON) TFT_Serial((uint8_t*)"t9.txt=\"ON\"");
			else if(g_mode == DEBUG_OFF) TFT_Serial((uint8_t*)"t9.txt=\"OFF\"");
		}
	}
}

void Create_DFF_In(uint8_t i)
{
	int j = 0, inspection_num = 0;
	uint8_t pinmap = 0;
	void(*Func)(void) = NULL;
	OS_Inspect *model1 = NULL;
	model1 = IN_SM_S928UB_INSERT;
	HAL_GPIO_WritePin(FET1_GPIO_Port, FET1_Pin, GPIO_PIN_RESET); //20 Ohm FET ON => Open inspection First
	HAL_Delay(1);
	inspection_num = model1[i].inspection_num; //IN

	for(j=0; j<inspection_num; j++)
	{
		Func = model1[i].pin[j].Func;
		pinmap = model1[i].pin[j].pinmap;

		DFF_Data(pinmap);
		Func();
	}

	HAL_Delay(1);
}

void Create_DFF_OUT(uint8_t i)
{
	int j = 0, inspection_num = 0;
	uint8_t pinmap = 0;
	void(*Func)(void) = NULL;
	OS_Inspect *model2 = NULL;
	model2 = OUT_SM_S928UB_INSERT;
	HAL_GPIO_WritePin(FET1_GPIO_Port, FET1_Pin, GPIO_PIN_RESET); //20 Ohm FET ON => Open inspection First
	HAL_Delay(1);
	inspection_num = model2[i].inspection_num; //IN

	for(j=0; j<inspection_num; j++)
	{
		Func = model2[i].pin[j].Func;
		pinmap = model2[i].pin[j].pinmap;

		DFF_Data(pinmap);
		Func();
	}

	HAL_Delay(1);
}

void DFF_Init_In(uint8_t i)
{

	int inspection_num = 0;
	void(*Func)(void) = NULL;
	OS_Inspect *model1 = NULL;
	model1 = IN_SM_S928UB_INSERT;
	HAL_GPIO_WritePin(FET1_GPIO_Port, FET1_Pin, GPIO_PIN_RESET); //20 Ohm FET ON => Open inspection First
	HAL_Delay(1);
	inspection_num = model1[i].inspection_num; //IN

	for(int j=0; j<inspection_num; j++)
	{
		Func = model1[i].pin[j].Func;
		DFF_Data(0x00);
		Func();
	}
}

void DFF_Init_OUT(uint8_t i)
{

	int inspection_num = 0;
	void(*Func)(void) = NULL;
	OS_Inspect *model2 = NULL;
	model2 = OUT_SM_S928UB_INSERT;
	HAL_GPIO_WritePin(FET1_GPIO_Port, FET1_Pin, GPIO_PIN_RESET); //20 Ohm FET ON => Open inspection First
	HAL_Delay(1);
	inspection_num = model2[i].inspection_num; //IN

	for(int j=0; j<inspection_num; j++)
	{
		Func = model2[i].pin[j].Func;
		DFF_Data(0x00);
		Func();
	}
}

void Vaccum_Push_Set(void)
{
	Push_Flag = pba_false;

	DFF_GND_Init();

	if(g_type == IN && count3 != 3)
	{
		Create_DFF_In(0);
		count3 = 3;
	}
	if(g_type == OUT && count3 != 3)
	{
		Create_DFF_OUT(0);
		count3 = 3;
	}
	if (g_type == ALL)
	{
		if(count1 == 0 && count3 != 3)
		{
			Create_DFF_In(0);
			count3 = 3;
		}
		else
		{	//LCD_ErrLog("count3: %d count1: %d ", count3,count1);
			Create_DFF_OUT(0);
			count2 =1;
		}
	}
	HAL_Delay(1);
}
void Auto_Clean(void)
{
	if((HAL_GPIO_ReadPin(LEFT_KEY_GPIO_Port, LEFT_KEY_Pin) == GPIO_PIN_RESET) && (HAL_GPIO_ReadPin(RIGHT_KEY_GPIO_Port, RIGHT_KEY_Pin) == GPIO_PIN_RESET) && (Push_Flag == pba_false) && (HAL_GPIO_ReadPin(MODE_KEY_GPIO_Port, MODE_KEY_Pin ) == GPIO_PIN_RESET))
		{
			HAL_Delay(10);
			while(HAL_GPIO_ReadPin(LEFT_KEY_GPIO_Port, LEFT_KEY_Pin) == GPIO_PIN_RESET || HAL_GPIO_ReadPin(RIGHT_KEY_GPIO_Port, RIGHT_KEY_Pin) == GPIO_PIN_RESET || (HAL_GPIO_ReadPin(MODE_KEY_GPIO_Port, MODE_KEY_Pin ) == GPIO_PIN_RESET));
			Beep(1,50);
			HAL_GPIO_WritePin(FET1_GPIO_Port, FET1_Pin, GPIO_PIN_SET); //Vaccum state 20 Ohm FET OFF
			LCD_BlueLog("**********AUTO CLEAN START**********\n");
			MAIN_ON;
			LCD_UsrLog("Main Cylinder ON\n");
			LCD_DbgLog("Main Sensor wait...\n");
		    _delay_ms(500);
		    CLEAN_ON;
			for(;;) //Sensor wait
			{
				if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_SET) break; //Sensor ON
				if(Rst_Flag == pba_true) break;
			}
			_delay_ms(2000);
			MAIN_OFF;
			HAL_Delay(500);
			CLEAN_OFF;
			LCD_UsrLog("Main Cylinder OFF\n");
			LCD_DbgLog("Main Sensor wait...\n");

			for(;;) //Sensor wait
			{
				if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET) break; //Sensor OFF
				if(Rst_Flag == pba_true) break;
			}
			g_clean++;
			LCD_BlueLog("********AUTO CLEAN COMPLETED********\n");
			HAL_Delay(500);
			g_rbuf[0] = (uint8_t)(g_clean & 0x000000FF);
			g_rbuf[1] = (uint8_t)((g_clean & 0x0000FF00) >> 8);
			g_rbuf[2] = (uint8_t)((g_clean & 0x00FF0000) >> 16);
			g_rbuf[3] = (uint8_t)((g_clean & 0xFF000000) >> 24);
			eep_256_burst_write(CLEAN_ADDRESS, g_rbuf, 4);
			sprintf((char*)g_buff, "t21.txt=\"%5ld\"", g_clean);
			TFT_Serial(g_buff);

			if(Rst_Flag == pba_true) return;
			Vaccum_Push_Set();
		}
}


void Key_input(void)
{
/*	if (g_type != IN && HAL_GPIO_ReadPin(MODE_KEY_GPIO_Port, MODE_KEY_Pin) == GPIO_PIN_SET&& mode_state == pba_false)
	{
		LCD_ErrLog("CHON TYPE = IN\n");
		_delay_ms(1500);
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);


	}*/

	if(g_key_on == KEY_ON)//insert key
	{
	if(((HAL_GPIO_ReadPin(RIGHT_KEY_GPIO_Port, RIGHT_KEY_Pin) == GPIO_PIN_RESET)) && Push_Flag == pba_false)
		{
			while(HAL_GPIO_ReadPin(RIGHT_KEY_GPIO_Port, RIGHT_KEY_Pin) == GPIO_PIN_RESET);

			Push_Flag = pba_true;

			if((g_test == DEBUG_ON) && (Debug_Flag == pba_false))
				TFT_Serial((uint8_t*)"page 1");

			VAC_ON;
			VAC1_ON;
			LCD_UsrLog("VACCUM ON\n");

			LCD_DbgLog("Press START Key\n");
		}

	while(Push_Flag == pba_true)
		{
			if(((HAL_GPIO_ReadPin(RIGHT_KEY_GPIO_Port, RIGHT_KEY_Pin) == GPIO_PIN_RESET)) && Push_Flag == pba_true)
			{
				HAL_Delay(10);

				while(HAL_GPIO_ReadPin(RIGHT_KEY_GPIO_Port, RIGHT_KEY_Pin) == GPIO_PIN_RESET);

				Beep(1,50);

				Push_Flag = pba_false;

				DFF_GND_Init();

				HAL_GPIO_WritePin(FET1_GPIO_Port, FET1_Pin, GPIO_PIN_SET); //Vaccum state 20 Ohm FET OFF

				LCD_BlueLog("**********INSPECTION START**********\n");

				MAIN_ON;

				HAL_GPIO_WritePin(NG_LED_GPIO_Port, NG_LED_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(OK_LED_GPIO_Port, OK_LED_Pin, GPIO_PIN_SET);

				HAL_GPIO_WritePin(LLED_NG_GPIO_Port, LLED_NG_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LLED_OK_GPIO_Port, LLED_OK_Pin, GPIO_PIN_SET);

				HAL_GPIO_WritePin(RLED_NG_GPIO_Port, RLED_NG_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(RLED_OK_GPIO_Port, RLED_OK_Pin, GPIO_PIN_SET);

				LCD_UsrLog("Main Cylinder ON\n");
				LCD_DbgLog("Main Sensor wait...\n");

				for(;;) //Sensor wait
				{
					if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_SET) break; //Sensor ON
					if(Rst_Flag == pba_true) break;
				}

				if(Rst_Flag == pba_true) return;

				DFF_GND_Init();
				_delay_ms(300);
				Start_Flag = pba_true;
				VAC_OFF;
				VAC1_OFF;
				break;
			}
			if(Rst_Flag == pba_true)
			{
				Push_Flag = pba_false;
			}
		}
	}

	if(g_key_on == KEY_OFF) //insert manual
	{

		if(g_type == IN && count1 == 0)
		{
			OS_STATE inspection_kind = OPEN;
			float resistor = 0;
			resistor = OS_Function(OPEN);
			if(inspection_kind == OPEN)
			{
				if(resistor < 200 && Push_Flag == pba_false)
				{
					count1 = 1;
				}
			}
		}

		if(g_type == OUT && count2 == 0)
			{
				OS_STATE inspection_kind = OPEN;
				float resistor = 0;
				resistor = OS_Function(OPEN);
				if(inspection_kind == OPEN)
				{
					if(resistor < 200 && Push_Flag == pba_false)
					{
						count2 = 1;
					}
				}
			}

		if(g_type == ALL)
		{
			OS_STATE inspection_kind = OPEN;
			float resistor = 0;
			resistor = OS_Function(OPEN);
			if(inspection_kind == OPEN)
			{

				if(resistor < 200 && Push_Flag == pba_false && count3 == 2)// && count2 == 0)
				{
					count2 = 1;
				}
				if(resistor < 200 && Push_Flag == pba_false && count1 == 0 && count2 == 0 && (count3 == 0 || count3 == 3))
				{
					DFF_GND_Init();
					DFF_Init_In(0);
					count3 = 1;
					HAL_Delay(10);
				}
				if(count3 == 1)
				{
					Create_DFF_OUT(0);
					count1 = 1;
					count3 = 2;
				}
				if(count1 == 1 && count2 == 0)
				{
					VAC_ON;
					VAC1_ON;
					sprintf((char*)g_buff,"xstr 5,65,105,90,5,GREEN,GRAY,1,1,1,\"OK\"");
					TFT_Serial(g_buff);
					sprintf((char*)g_buff,"xstr 110,65,117,90,2,YELLOW,GRAY,1,1,1,\"IN\"");
					TFT_Serial(g_buff);
					sprintf((char*)g_buff,"xstr 227,65,105,90,5,GREEN,GRAY,1,1,1,\"OK\"");
					TFT_Serial(g_buff);
					sprintf((char*)g_buff,"xstr 5,170,105,90,5,RED,GRAY,1,1,1,\"NG\"");
					TFT_Serial(g_buff);
					sprintf((char*)g_buff,"xstr 110,170,117,90,2,YELLOW,GRAY,1,1,1,\"OUT\"");
					TFT_Serial(g_buff);
					sprintf((char*)g_buff,"xstr 227,170,105,90,5,RED,GRAY,1,1,1,\"NG\"");
					TFT_Serial(g_buff);
				}
			}
		}
		if(Push_Flag == pba_false && ((count1 == 1  && g_type == IN )||(count2 == 1  && g_type == OUT ) ))
				{
					count1 = 0;
					count2 = 0;
					count3 = 0;
					Vaccum_Push = 1;
				}

		if(Push_Flag == pba_false && count1 == 1 && count2 == 1 && g_type == ALL)
				{
					count1 = 0;
					count2 = 0;
					count3 = 0;
					Vaccum_Push = 1;
				}

		if(Vaccum_Push == 1 && Push_Flag == pba_false)
		{
			Vaccum_Push = 0;
			Push_Flag = pba_true;

			if((g_test == DEBUG_ON) && (Debug_Flag == pba_false))
				TFT_Serial((uint8_t*)"page 1");

			VAC_ON;
			VAC1_ON;
			LCD_UsrLog("VACCUM ON\n");

			Beep(1,50);
			LCD_DbgLog("Press L.START + R.START Key\n");
		}

		if(((HAL_GPIO_ReadPin(RIGHT_KEY_GPIO_Port, RIGHT_KEY_Pin) == GPIO_PIN_RESET)) && Push_Flag == pba_true)
		{
			HAL_Delay(10);

			while(HAL_GPIO_ReadPin(RIGHT_KEY_GPIO_Port, RIGHT_KEY_Pin) == GPIO_PIN_RESET);

			Beep(1,50);

			HAL_GPIO_WritePin(FET1_GPIO_Port, FET1_Pin, GPIO_PIN_SET); //Vaccum state 20 Ohm FET OFF

			LCD_BlueLog("**********INSPECTION START**********\n");

			MAIN_ON;

			HAL_GPIO_WritePin(NG_LED_GPIO_Port, NG_LED_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(OK_LED_GPIO_Port, OK_LED_Pin, GPIO_PIN_SET);

			HAL_GPIO_WritePin(LLED_NG_GPIO_Port, LLED_NG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LLED_OK_GPIO_Port, LLED_OK_Pin, GPIO_PIN_SET);

			HAL_GPIO_WritePin(RLED_NG_GPIO_Port, RLED_NG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RLED_OK_GPIO_Port, RLED_OK_Pin, GPIO_PIN_SET);

			LCD_UsrLog("Main Cylinder ON\n");
			LCD_DbgLog("Main Sensor wait...\n");

			for(;;) //Sensor wait
			{
				if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_SET) break; //Sensor ON
				if(Rst_Flag == pba_true) break;
			}

			if(Rst_Flag == pba_true) return;

			DFF_GND_Init();
			_delay_ms(700);
			Start_Flag = pba_true;
			VAC_OFF;
			VAC1_OFF;
		}
	}
}

#ifdef SPEC_1M

static float R_Function(void)
{
	float voltage = 0, R_value = 0;
	uint32_t res = 0, adc_sum = 0;
	int i = 0, time = 60;

	for(i=0; i<5; i++)
	{
		if(i==0)	//20ohm
		{
			res = 20;
			HAL_GPIO_WritePin(FET5_GPIO_Port, FET5_Pin, GPIO_PIN_SET); //200K OFF
			HAL_GPIO_WritePin(FET1_GPIO_Port, FET1_Pin, GPIO_PIN_RESET); //20 ON
		}
		else if(i==1)	//200ohm
		{
			res = 200;
			HAL_GPIO_WritePin(FET1_GPIO_Port, FET1_Pin, GPIO_PIN_SET); //20 OFF
			HAL_GPIO_WritePin(FET2_GPIO_Port, FET2_Pin, GPIO_PIN_RESET); //200 ON
		}
		else if(i==2)	//2Kohm
		{
			res = 2000;
			HAL_GPIO_WritePin(FET2_GPIO_Port, FET2_Pin, GPIO_PIN_SET); //200 OFF
			HAL_GPIO_WritePin(FET3_GPIO_Port, FET3_Pin, GPIO_PIN_RESET); //2K ON
		}
		else if(i==3)	//20Kohm
		{
			res = 20000;
			HAL_GPIO_WritePin(FET3_GPIO_Port, FET3_Pin, GPIO_PIN_SET); //2K OFF
			HAL_GPIO_WritePin(FET4_GPIO_Port, FET4_Pin, GPIO_PIN_RESET); //20K ON
		}
		else if(i==4)	//200Kohm
		{
			res = 200000;
			HAL_GPIO_WritePin(FET4_GPIO_Port, FET4_Pin, GPIO_PIN_SET); //20K OFF
			HAL_GPIO_WritePin(FET5_GPIO_Port, FET5_Pin, GPIO_PIN_RESET); //200K ON
		}
		HAL_Delay(1);

		for(int j=0; j<time; j++)
		{
			HAL_ADC_Start(&hadc1);

			HAL_ADC_PollForConversion(&hadc1, 500);
			adc_sum += HAL_ADC_GetValue(&hadc1);

			HAL_ADC_Stop(&hadc1);
		//	HAL_Delay(1);

		}
		voltage = ((float)(adc_sum/time)/4095)*3.3;
		adc_sum = 0;

		if(voltage < 3.0)
					break;
	}

	HAL_GPIO_WritePin(FET1_GPIO_Port, FET1_Pin, GPIO_PIN_SET); //20 OFF
	HAL_GPIO_WritePin(FET2_GPIO_Port, FET2_Pin, GPIO_PIN_SET); //200 OFF
	HAL_GPIO_WritePin(FET3_GPIO_Port, FET3_Pin, GPIO_PIN_SET); //2K OFF
	HAL_GPIO_WritePin(FET4_GPIO_Port, FET4_Pin, GPIO_PIN_SET); //20K OFF
	HAL_GPIO_WritePin(FET5_GPIO_Port, FET5_Pin, GPIO_PIN_SET); //200K ON

	if((i==5) && (voltage >= 3.0)) R_value = 2000000;
	else R_value = (((voltage * res)/(3.3-voltage))-80); //20

	if(R_value < 0) R_value = 0;

	return R_value;
}

static float OS_Function(OS_STATE value)
{
	float voltage = 0, R_value = 0;
	uint32_t adc_sum = 0;
	int time = 0;

	if(value == OPEN) time = 90;
	else if(value == SHORT) time = 70;
	else if(value == Dbg) time = 10;

	for(int i=0; i<time; i++)
	{
		HAL_ADC_Start(&hadc1);

		HAL_ADC_PollForConversion(&hadc1, 500);
		adc_sum += HAL_ADC_GetValue(&hadc1);

		HAL_ADC_Stop(&hadc1);
	}

	voltage = ((float)(adc_sum/time)/4095)*3.3;

	if(value == OPEN || value == Dbg)
	{
		if(voltage > 3.0) R_value = 200;
		//else R_value = (((voltage * 20)/(3.3-voltage))-80) * 0.7; //20
		else R_value = (((voltage * 20)/(3.3-voltage))-84) * 0.7; //20
	}
	else if(value == SHORT)
	{
		if(voltage > 3.0) R_value = 2000000;
		else R_value = ((voltage * 200000)/(3.3-voltage)) * 0.72; //200K, 28%
	}

	if(R_value < 0) R_value = 0;

	return R_value;
}

static void Fet_spec(uint8_t fet, uint8_t status){
	if(fet==2){
		if(status == FET_ON){
			HAL_GPIO_WritePin(FET1_GPIO_Port, FET1_Pin, GPIO_PIN_SET); //20 Ohm FET OFF
			HAL_Delay(1);
			HAL_GPIO_WritePin(FET5_GPIO_Port,FET5_Pin, GPIO_PIN_RESET); //200K FET ON
			HAL_Delay(1);
		}
		else {
			HAL_GPIO_WritePin(FET5_GPIO_Port, FET5_Pin, GPIO_PIN_SET); //200K OFF
			HAL_Delay(1);
			HAL_GPIO_WritePin(FET1_GPIO_Port, FET1_Pin, GPIO_PIN_RESET); //20 ON
			HAL_Delay(1);
		}
	}
	else if(fet==1){
		if(status == FET_ON){
			HAL_GPIO_WritePin(FET5_GPIO_Port, FET5_Pin, GPIO_PIN_RESET); //200K ON
		}
		else if(status == FET_OFF){
			HAL_GPIO_WritePin(FET5_GPIO_Port, FET5_Pin, GPIO_PIN_SET); //200K OFF
		}
	}
}

#endif
#ifdef SPEC_10M

static float R_Function(void)
{
	float voltage = 0, R_value = 0;
	uint32_t res = 0, adc_sum = 0;
	int i = 0, time = 55; // 65

	for(i=0; i<6; i++)
	{

		if(i==0)	//20ohm
		{
			res = 20;
			HAL_GPIO_WritePin(FET6_GPIO_Port, FET6_Pin, GPIO_PIN_SET); //200K OFF
			HAL_GPIO_WritePin(FET1_GPIO_Port, FET1_Pin, GPIO_PIN_RESET); //20 ON
		}
		else if(i==1)	//200ohm
		{
			res = 200;
			HAL_GPIO_WritePin(FET1_GPIO_Port, FET1_Pin, GPIO_PIN_SET); //20 OFF
			HAL_GPIO_WritePin(FET2_GPIO_Port, FET2_Pin, GPIO_PIN_RESET); //200 ON
		}
		else if(i==2)	//2Kohm
		{
			res = 2000;
			HAL_GPIO_WritePin(FET2_GPIO_Port, FET2_Pin, GPIO_PIN_SET); //200 OFF
			HAL_GPIO_WritePin(FET3_GPIO_Port, FET3_Pin, GPIO_PIN_RESET); //2K ON
		}
		else if(i==3)	//20Kohm
		{
			res = 20000;
			HAL_GPIO_WritePin(FET3_GPIO_Port, FET3_Pin, GPIO_PIN_SET); //2K OFF
			HAL_GPIO_WritePin(FET4_GPIO_Port, FET4_Pin, GPIO_PIN_RESET); //20K ON
		}
		else if(i==4)	//200Kohm
		{
			res = 200000;
			HAL_GPIO_WritePin(FET4_GPIO_Port, FET4_Pin, GPIO_PIN_SET); //20K OFF
			HAL_GPIO_WritePin(FET5_GPIO_Port, FET5_Pin, GPIO_PIN_RESET); //200K ON
		}
		else if(i==5)	//200Kohm
		{
			res = 500000;
			HAL_GPIO_WritePin(FET5_GPIO_Port, FET5_Pin, GPIO_PIN_SET); //20K OFF
			HAL_GPIO_WritePin(FET6_GPIO_Port, FET6_Pin, GPIO_PIN_RESET); //500K ON
			HAL_Delay(1);

				}
		HAL_Delay(1);

		for(int j=0; j<time; j++)
		{
			HAL_ADC_Start(&hadc1);

			HAL_ADC_PollForConversion(&hadc1, 500);
			adc_sum += HAL_ADC_GetValue(&hadc1);

			HAL_ADC_Stop(&hadc1);
		//	HAL_Delay(1);

		}
		voltage = ((float)(adc_sum/time)/4095)*3.3;
		adc_sum = 0;

		if(voltage < 3.0)
					break;
	}

	HAL_GPIO_WritePin(FET1_GPIO_Port, FET1_Pin, GPIO_PIN_SET); //20 OFF
	HAL_GPIO_WritePin(FET2_GPIO_Port, FET2_Pin, GPIO_PIN_SET); //200 OFF
	HAL_GPIO_WritePin(FET3_GPIO_Port, FET3_Pin, GPIO_PIN_SET); //2K OFF
	HAL_GPIO_WritePin(FET4_GPIO_Port, FET4_Pin, GPIO_PIN_SET); //20K OFF
	HAL_GPIO_WritePin(FET5_GPIO_Port, FET5_Pin, GPIO_PIN_SET); //200K OFF
	HAL_GPIO_WritePin(FET6_GPIO_Port, FET6_Pin, GPIO_PIN_SET); //500K OFF

	if((i==5) && (voltage >= 3.185)) R_value = 20000000;
	else R_value = (((voltage * res)/(3.3-voltage))-80); //20
	if(R_value>10000000)
	{
		R_value = R_value-2000000;
	}
	if(R_value < 0) R_value = 0;

	return R_value;
}
static float OS_Function(OS_STATE value)
{

	float voltage = 0, R_value = 0;
	uint32_t adc_sum = 0;
	int time = 0;

	if(value == OPEN) time = 70; //90
	else if(value == SHORT) time = 60; //70
	else if(value == Dbg) time = 10;


	if(value == SHORT)
	{
		HAL_GPIO_WritePin(FET1_GPIO_Port, FET1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(FET6_GPIO_Port, FET6_Pin, GPIO_PIN_RESET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(FET1_GPIO_Port, FET1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(FET6_GPIO_Port, FET6_Pin, GPIO_PIN_SET);
		HAL_Delay(1);
	}


	for(int i=0; i<time; i++)
	{
		HAL_ADC_Start(&hadc1);

		HAL_ADC_PollForConversion(&hadc1, 500);
		adc_sum += HAL_ADC_GetValue(&hadc1);

		HAL_ADC_Stop(&hadc1);
	}

	voltage = ((float)(adc_sum/time)/4095)*3.3;

	if(value == OPEN || value == Dbg)
	{
		if(voltage > 3.0) R_value = 200;
		//else R_value = (((voltage * 20)/(3.3-voltage))-80) * 0.7; //20
		else R_value = (((voltage * 20)/(3.3-voltage))-84) * 0.7; //20
	}
	else if(value == SHORT)
	{
		if(voltage > 3.185) R_value = 20000000;
		else R_value = ((voltage * 500000)/(3.3-voltage)*0.72); //200K, 28%
	}

	if(R_value < 0) R_value = 0;

	return R_value;
}

static void Fet_spec(uint8_t fet, uint8_t status){
	if(fet==2){
		if(status == FET_ON){
			HAL_GPIO_WritePin(FET1_GPIO_Port, FET1_Pin, GPIO_PIN_SET); //20 Ohm FET OFF
			HAL_Delay(1);
			HAL_GPIO_WritePin(FET6_GPIO_Port,FET6_Pin, GPIO_PIN_RESET); //200K FET ON
			HAL_Delay(1);
		}
		else if(status == FET_OFF){
			HAL_GPIO_WritePin(FET6_GPIO_Port, FET6_Pin, GPIO_PIN_SET); //200K OFF
			HAL_Delay(1);
			HAL_GPIO_WritePin(FET1_GPIO_Port, FET1_Pin, GPIO_PIN_RESET); //20 ON
			HAL_Delay(1);
		}

	}
	else if(fet==1){
		if(status == FET_ON){
			HAL_GPIO_WritePin(FET6_GPIO_Port,FET6_Pin, GPIO_PIN_RESET); //200K FET ON
		}
		else if(status == FET_OFF){
			HAL_GPIO_WritePin(FET6_GPIO_Port, FET6_Pin, GPIO_PIN_SET);  //200K FET OFF
		}
	}
}

#endif


void Start_Task(void)
{

			if(Start_Flag == pba_true)
			{

				int i = 0, j = 0, k = 0, total_num = 0, short_index = 0, inspection_num = 0;
				OS_STATE inspection_kind = OPEN;
				uint8_t pinmap = 0;
				void(*Func)(void) = NULL;
				char *string = NULL;
				float resistor = 0;
				OS_Inspect *model1 = NULL, *model2 = NULL;

				gmode_count = 0;

				In_ostate = pba_true;
				In_sstate = pba_true;
				Out_ostate = pba_true;
				Out_sstate = pba_true;

				Start_Flag = pba_false;

				total_num = 128; //total num.
				short_index = 70; // short start index num

				model1 = IN_SM_S928UB;
				model2 = OUT_SM_S928UB;

				g_tick_timer1 = HAL_GetTick();

				HAL_GPIO_WritePin(FET1_GPIO_Port, FET1_Pin, GPIO_PIN_RESET); //20 Ohm FET ON => Open inspection First
				HAL_Delay(1);

				if(g_type == OUT)
				{
							for(i=0; i<total_num; i++)
							{
								if(i == short_index)
								{
		//							HAL_GPIO_WritePin(FET1_GPIO_Port, FET1_Pin, GPIO_PIN_SET); //20 Ohm FET OFF
		//							HAL_Delay(1);
		//							HAL_GPIO_WritePin(FET6_GPIO_Port,FET6_Pin, GPIO_PIN_RESET); //200K FET ON
		//							HAL_Delay(1);
									Fet_spec(2,FET_ON);
								}

								string = model2[i].Group_Name; //IN
								inspection_num = model2[i].inspection_num; //IN
								inspection_kind = model2[i].inspection_kind; //IN

								for(j=0; j<inspection_num; j++)
								{
									Func = model2[i].pin[j].Func;
									pinmap = model2[i].pin[j].pinmap;

									DFF_Data(pinmap);
									Func();
								}

		//						HAL_Delay(1);

								if(inspection_kind == OPEN)
								{
									resistor = OS_Function(OPEN);

									if(resistor > OPEN_SPEC)
									{
										Out_ostate = pba_false;
										if(g_mode == 0) LCD_ErrLog("[%s]:%.0f[R] OPEN\n", string, resistor);
										//k++;
										//if(k==10){LCD_ErrLog("ERROR TOO MANY OPEN\n");k=0;i = short_index - 1;}
									}
									else
									{
										if (resistor > 10)
										{
											resistor = resistor -10;
																	//In_ostate = pba_true;
											sprintf((char*)g_buff, "[Ok]:[%s]:%.0f[R] OPEN\r\n", string, resistor);
											Debug_serial((char*)g_buff);
										}
										else
										{
											sprintf((char*)g_buff, "[Ok]:[%s]:%.0f[R] OPEN\r\n", string, resistor);
											Debug_serial((char*)g_buff);
										}
									}

								}
								else if(inspection_kind == SHORT)
								{
									//HAL_Delay(1);
									resistor = OS_Function(SHORT);

									if(resistor < SHORT_SPEC)
									{
										resistor = R_Function();
										if(resistor < SHORT_SPEC)
										{
											Out_sstate = pba_false;//short ÀúÇ×°ª ĂøÁ¤
											if(g_mode == 0) LCD_ErrLog("[%s]:%.0f[R] SHORT\n", string, resistor);
		//									HAL_GPIO_WritePin(FET6_GPIO_Port, FET6_Pin, GPIO_PIN_RESET);
											Fet_spec(1,FET_ON);
										}
										else
										{
											sprintf((char*)g_buff, "[ Ok]:[%s]:%.0f[R] SHORT\r\n", string, resistor);
											Debug_serial((char*)g_buff);
										}
										//HAL_GPIO_WritePin(FET6_GPIO_Port, FET6_Pin, GPIO_PIN_RESET); //200K On
									}
									else
									{
										sprintf((char*)g_buff, "[ Ok]:[%s]:%.0f[R] SHORT\r\n", string, resistor);
										Debug_serial((char*)g_buff);
									}

		//							HAL_GPIO_WritePin(FET6_GPIO_Port, FET6_Pin, GPIO_PIN_RESET);
									Fet_spec(1,FET_ON);
									HAL_Delay(1);
								}

								for(j=0; j<inspection_num; j++)
								{
									Func = model2[i].pin[j].Func;

									DFF_Data(0x00);
									Func();

								}

								if(Rst_Flag == pba_true) return;
							}

		//					HAL_GPIO_WritePin(FET6_GPIO_Port, FET6_Pin, GPIO_PIN_SET); //200K OFF
		//					HAL_Delay(1);
		//					HAL_GPIO_WritePin(FET1_GPIO_Port, FET1_Pin, GPIO_PIN_RESET); //20 ON
		//					HAL_Delay(1);
							Fet_spec(2,FET_OFF);
						}

						if(g_type == IN)
						{
							while(1)
							{

							for(i=0; i<total_num; i++)
							{
								if(i == short_index)
								{
		/*							HAL_GPIO_WritePin(FET1_GPIO_Port, FET1_Pin, GPIO_PIN_SET); //20 Ohm FET OFF
									HAL_Delay(1);
									HAL_GPIO_WritePin(FET6_GPIO_Port,FET6_Pin, GPIO_PIN_RESET); //200K FET ON
									HAL_Delay(1);*/
									Fet_spec(2,FET_ON);
								}

								string = model1[i].Group_Name; //OUT
								inspection_num = model1[i].inspection_num; //OUT
								inspection_kind = model1[i].inspection_kind; //OUT

								for(j=0; j<inspection_num; j++)
								{
									Func = model1[i].pin[j].Func;
									pinmap = model1[i].pin[j].pinmap;

									DFF_Data(pinmap);
									Func();
								}

		//						HAL_Delay(1);

								if(inspection_kind == OPEN)
								{
									resistor = OS_Function(OPEN);

									if(resistor > OPEN_SPEC)
									{
										In_ostate = pba_false;
										if(g_mode == 0) LCD_ErrLog("[%s]:IN 3.3[V] %.0f[R] OPEN\n", string, resistor);
										break;
										/*k++;
										if(k==7){LCD_ErrLog("ERROR TOO MANY OPEN\n");k=0;i = short_index - 1;}*/
									}
									else
									{
										if (resistor > 10)
										{
										resistor = resistor -10;
										//In_ostate = pba_true;
										sprintf((char*)g_buff, "[Ok]:[%s]:IN 3.3[V] %.0f[R] OPEN\r\n", string, resistor);
										LCD_OkLog("[%s]:IN 3.3[V] %.0f[R] OPEN\r\n", string, resistor);

										Debug_serial((char*)g_buff);
										}
										else
										{
										sprintf((char*)g_buff, "[Ok]:[%s]:IN 3.3[V] %.0f[R] OPEN\r\n", string, resistor);
										LCD_OkLog("[%s]:IN 3.3[V] %.0f[R] OPEN\r\n", string, resistor);
										Debug_serial((char*)g_buff);
										}
									}

								}

								else if(inspection_kind == SHORT)
								{
									//HAL_Delay(1);
									resistor = OS_Function(SHORT);

									if(resistor < SHORT_SPEC)
									{
										resistor = R_Function();
										if(resistor < SHORT_SPEC)
										{
											In_sstate = pba_false;
											if(g_mode == 0) LCD_ErrLog("[%s]:IN 3.3[V] %.0f[R]SHORT\n", string, resistor);
											break;
											//HAL_GPIO_WritePin(FET5_GPIO_Port, FET5_Pin, GPIO_PIN_RESET);
										}
										else
										{
											sprintf((char*)g_buff, "[Ok]:[%s]:IN 3.3[V] %.0f[R]SHORT\r\n", string, resistor);
											LCD_OkLog("[%s]:IN 3.3[V] %.0f[R]SHORT\r\n", string, resistor);
											Debug_serial((char*)g_buff);
										}
										//HAL_GPIO_WritePin(FET6_GPIO_Port, FET6_Pin, GPIO_PIN_RESET); //200K On
									}
									else
									{
										sprintf((char*)g_buff, "[Ok]:[%s]:IN 3.3[V] %.0f[R]SHORT\r\n", string, resistor);
										LCD_OkLog("[%s]:IN 3.3[V] %.0f[R]SHORT\r\n", string, resistor);
										Debug_serial((char*)g_buff);
									}

								HAL_GPIO_WritePin(FET6_GPIO_Port, FET6_Pin, GPIO_PIN_RESET);
								HAL_Delay(1);
								}

								//sprintf((char*)g_buff, "[%s]:%.0f[OHM]\r\n", string, resistor);
								//Debug_serial((char*)g_buff);

								for(j=0; j<inspection_num; j++)
								{
									Func = model1[i].pin[j].Func;

									DFF_Data(0x00);
									Func();
									//HAL_Delay(5);
								}

								if(Rst_Flag == pba_true) return;
							}

		/*					HAL_GPIO_WritePin(FET6_GPIO_Port, FET6_Pin, GPIO_PIN_SET); //200K OFF
							HAL_Delay(1);
							HAL_GPIO_WritePin(FET1_GPIO_Port, FET1_Pin, GPIO_PIN_RESET); //20 ON
							HAL_Delay(1);*/

							Fet_spec(2,FET_OFF);
							temp = (g_tick_timer2-g_tick_timer1)/1000;
							/*giay = temp ;
							if(giay > 59)
								{	phut = phut + 1 ;
									giay = giay - 59  ;
								}
							if (phut > 59)
							{
								gio = gio + 1 ;
								phut = phut - 59;
							}


							char distime [20];

							sprintf(distime,"02%d:02%d:02%d",gio,phut,giay);*/
							sprintf((char*)g_buff, "t9.txt=\"%s\ [s]", temp);
							TFT_Serial(g_buff);

							if (In_sstate == pba_false || In_ostate == pba_false ) break;
							}
						}



						if(g_type == ALL)
						{
		//--------------------------------------------------------IN-------------------------------------------------------------------

							for(i=0; i<total_num; i++)
							{
								if(i == short_index)
								{
		//							HAL_GPIO_WritePin(FET1_GPIO_Port, FET1_Pin, GPIO_PIN_SET); //20 Ohm FET OFF
		//							HAL_Delay(1);
		//							HAL_GPIO_WritePin(FET6_GPIO_Port,FET6_Pin, GPIO_PIN_RESET); //200K FET ON
		//							HAL_Delay(1);
									Fet_spec(2,FET_ON);
								}

								string = model1[i].Group_Name; //IN
								inspection_num = model1[i].inspection_num; //IN
								inspection_kind = model1[i].inspection_kind; //IN

								for(j=0; j<inspection_num; j++)
								{
									Func = model1[i].pin[j].Func;
									pinmap = model1[i].pin[j].pinmap;

									DFF_Data(pinmap);
									Func();
								}

		//						HAL_Delay(1);

								if(inspection_kind == OPEN)
								{
									resistor = OS_Function(OPEN);

									if(resistor > OPEN_SPEC)
									{
										In_ostate = pba_false;
										if(g_mode == 0) LCD_ErrLog("[%s]:%.0f[R] OPEN\n", string, resistor);
										k++;
										if(k==10){LCD_ErrLog("ERROR TOO MANY OPEN\n");k=0;i = short_index - 1;}
									}
									else
									{
										if (resistor > 10)
										{
										resistor = resistor -10;
										//In_ostate = pba_true;
										sprintf((char*)g_buff, "[ Ok]:[%s]:%.0f[R] OPEN\r\n", string, resistor);
										Debug_serial((char*)g_buff);
										}
										else
										{
										sprintf((char*)g_buff, "[ Ok]:[%s]:%.0f[R] OPEN\r\n", string, resistor);
										Debug_serial((char*)g_buff);
										}
									}
								}
								else if(inspection_kind == SHORT)
								{
									//HAL_Delay(1);
									resistor = OS_Function(SHORT);

									if(resistor < SHORT_SPEC)
									{
										resistor = R_Function();
										if(resistor < SHORT_SPEC)
										{
											In_sstate = pba_false;//short ÀúÇ×°ª ĂøÁ¤
											if(g_mode == 0) LCD_ErrLog("[%s]:%.0f[R] SHORT\n", string, resistor);
		//									HAL_GPIO_WritePin(FET6_GPIO_Port, FET6_Pin, GPIO_PIN_RESET);
											Fet_spec(1,FET_ON);
										}
										else
										{
											sprintf((char*)g_buff, "[ Ok]:[%s]:%.0f[R] SHORT\r\n", string, resistor);
											Debug_serial((char*)g_buff);
										}
										//HAL_GPIO_WritePin(FET6_GPIO_Port, FET6_Pin, GPIO_PIN_RESET); //200K On
									}
									else
									{
										sprintf((char*)g_buff, "[ Ok]:[%s]:%.0f[R] SHORT\r\n", string, resistor);
										Debug_serial((char*)g_buff);
									}

		//							HAL_GPIO_WritePin(FET6_GPIO_Port, FET6_Pin, GPIO_PIN_RESET);
									Fet_spec(1,FET_ON);
									HAL_Delay(1);
								}

								//sprintf((char*)g_buff, "[%s]:%.0f[OHM]\r\n", string, resistor);
								//Debug_serial((char*)g_buff);

								for(j=0; j<inspection_num; j++)
								{
									Func = model1[i].pin[j].Func;

									DFF_Data(0x00);
									Func();
								}

								if(Rst_Flag == pba_true) return;
							}

		//					HAL_GPIO_WritePin(FET6_GPIO_Port, FET6_Pin, GPIO_PIN_SET); //200K OFF
		//					HAL_Delay(1);
		//					HAL_GPIO_WritePin(FET1_GPIO_Port, FET1_Pin, GPIO_PIN_RESET); //20 ON
		//					HAL_Delay(1);
							Fet_spec(2,FET_OFF);


		//-------------------------------------------------------------OUT-------------------------------------------------------------------------------------
							for(i=0; i<total_num; i++)
							{
								if(i == short_index)
								{
		//							HAL_GPIO_WritePin(FET1_GPIO_Port, FET1_Pin, GPIO_PIN_SET); //20 Ohm FET OFF
		//							HAL_Delay(1);
		//							HAL_GPIO_WritePin(FET6_GPIO_Port, FET6_Pin, GPIO_PIN_RESET); //200K FET ON
		//							HAL_Delay(1);
									Fet_spec(2,FET_ON);
								}

								string = model2[i].Group_Name; //IN
								inspection_num = model2[i].inspection_num; //IN
								inspection_kind = model2[i].inspection_kind; //IN

								for(j=0; j<inspection_num; j++)
								{
									Func = model2[i].pin[j].Func;
									pinmap = model2[i].pin[j].pinmap;

									DFF_Data(pinmap);
									Func();
								}

								//HAL_Delay(1);

								if(inspection_kind == OPEN)
								{
									resistor = OS_Function(OPEN);

									if(resistor > OPEN_SPEC)
									{
										Out_ostate = pba_false;
										if(g_mode == 0) LCD_ErrLog("[%s]:%.0f[R] OPEN\n", string, resistor);
										k++;
										if(k==10){LCD_ErrLog("ERROR TOO MANY OPEN\n");k=0;i = short_index - 1;}
									}
									else
									{
										if (resistor > 10)
										{
										resistor = resistor -10;
										//In_ostate = pba_true;
										sprintf((char*)g_buff, "[ Ok]:[%s]:%.0f[R] OPEN\r\n", string, resistor);
										Debug_serial((char*)g_buff);
										}
										else
										{
										sprintf((char*)g_buff, "[ Ok]:[%s]:%.0f[R] OPEN\r\n", string, resistor);
										Debug_serial((char*)g_buff);
										}
									}

								}
								else if(inspection_kind == SHORT)
								{
									//HAL_Delay(1);
									resistor = OS_Function(SHORT);

									if(resistor < SHORT_SPEC)
									{
										resistor = R_Function();
										if(resistor < SHORT_SPEC)
										{
											Out_sstate = pba_false;//short ÀúÇ×°ª ĂøÁ¤
											if(g_mode == 0) LCD_ErrLog("[%s]:%.0f[R] SHORT\n", string, resistor);
		//									HAL_GPIO_WritePin(FET6_GPIO_Port, FET6_Pin, GPIO_PIN_RESET);
											Fet_spec(1,FET_ON);
										}
										else
										{
											sprintf((char*)g_buff, "[ Ok]:[%s]:%.0f[R] SHORT\r\n", string, resistor);
											Debug_serial((char*)g_buff);
										}
										//HAL_GPIO_WritePin(FET6_GPIO_Port, FET6_Pin, GPIO_PIN_RESET);
									}
									else
									{
										sprintf((char*)g_buff, "[ Ok]:[%s]:%.0f[R] SHORT\r\n", string, resistor);
										Debug_serial((char*)g_buff);
									}

		//							HAL_GPIO_WritePin(FET6_GPIO_Port, FET6_Pin, GPIO_PIN_RESET);
									Fet_spec(1,FET_ON);
									HAL_Delay(1);
								}


								//sprintf((char*)g_buff, "[%s]:%.0f[OHM]\r\n", string, resistor);
								//Debug_serial((char*)g_buff);

								for(j=0; j<inspection_num; j++)
								{
									Func = model2[i].pin[j].Func;

									DFF_Data(0x00);
									Func();
								}

								if(Rst_Flag == pba_true) return;
							}

		//					HAL_GPIO_WritePin(FET6_GPIO_Port, FET6_Pin, GPIO_PIN_SET); //200K OFF
		//					HAL_Delay(1);
		//					HAL_GPIO_WritePin(FET1_GPIO_Port, FET1_Pin, GPIO_PIN_RESET); //20 ON
		//					HAL_Delay(1);
							Fet_spec(2,FET_OFF);

						}
						if((g_test == DEBUG_ON) && (Debug_Flag == pba_false)) Test_Result_Flag = pba_true;	//TEST¸ðµå °á°ú
						else Result_Flag = pba_true;
					}


	}


void Result_Task(void)
{
	if(Result_Flag == pba_true)
	{
		//LCD_OkLog("Vao result task");
		Result_Flag = pba_false;

		if(g_type == IN)
		{
			/*if(In_ostate == pba_true) LCD_OkLog("1.IN Open Inspection => OK[10R]\n");
			else LCD_ErrLog("1.IN Open Inspection => NG[10R]\n");

			if(In_sstate == pba_true) LCD_OkLog("1.IN Short Inspection => OK[10MR]\n");
			else LCD_ErrLog("1.IN Short Inspection => NG[10MR]\n");*/
		}

		else if(g_type == OUT)
		{
			if(Out_ostate == pba_true) LCD_OkLog("2.OUT Open Inspection => OK[10R]\n");
			else LCD_ErrLog("2.OUT Open Inspection => NG[10R]\n");

			if(Out_sstate == pba_true) LCD_OkLog("2.OUT Short Inspection => OK[10MR]\n");
			else LCD_ErrLog("2.OUT Short Inspection => NG[10MR]\n");
		}
		else if(g_type == ALL)
		{
			if(In_ostate == pba_true) LCD_OkLog("1.IN Open Inspection => OK[10R]\n");
			else LCD_ErrLog("1.IN Open Inspection => NG[10R]\n");
			if(In_sstate == pba_true) LCD_OkLog("1.IN Short Inspection => OK[10MR]\n");
			else LCD_ErrLog("1.IN Short Inspection => NG[10MR]\n");

			if(Out_ostate == pba_true) LCD_OkLog("2.OUT Open Inspection => OK[10R]\n");
			else LCD_ErrLog("2.OUT Open Inspection => NG[10R]\n");
			if(Out_sstate == pba_true) LCD_OkLog("2.OUT Short Inspection => OK[10MR]\n");
			else LCD_ErrLog("2.OUT Short Inspection => NG[10MR]\n");
		}


		g_tick_timer2 = HAL_GetTick();
		LCD_UsrLog("Tact time : %.2f[sec]\n", (float)(g_tick_timer2 - g_tick_timer1)/1000);



		DFF_Floating_Init();

		LCD_BlueLog("********INSPECTION COMPLETED********\n");

		_delay_ms(300);

		if(Rst_Flag == pba_true) return;

		pba_bool In_state = In_ostate & In_sstate;
		pba_bool Out_state = Out_ostate & Out_sstate;
		static int lock_count = 0;

		if(g_type == IN) //In mode
		{
			in_sum_count = in_ok_count + in_ng_count;
			total_sum_count = in_ok_count + in_ng_count + out_ok_count + out_ng_count;

			if(In_state == pba_true) //Left ok
			{
				lock_count = 0;

				Beep(2,50);

				HAL_GPIO_WritePin(OK_LED_GPIO_Port, OK_LED_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LLED_OK_GPIO_Port, LLED_OK_Pin, GPIO_PIN_RESET);

				//Result_Display(pba_true, pba_none);
				in_ok_count++;
				total_ok_count = in_ok_count + out_ok_count;

				if(g_mark == MARK_ON)
				{
					LMARK_ON;
					HAL_Delay(300);
					LMARK_OFF;
				}
			}


			else
			{
				HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(NG_LED_GPIO_Port, NG_LED_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LLED_NG_GPIO_Port, LLED_NG_Pin, GPIO_PIN_RESET);

				//Result_Display(pba_false, pba_none);
				in_ng_count++;
				total_ng_count = in_ng_count + out_ng_count;

				if(g_error == DEBUG_ON) if(lock_count < 5) lock_count++;
			}


		}
// =================================== MARK pen=============================================================

/*
		MAIN_OFF;
		LCD_UsrLog("Main Cylinder OFF\n");
		LCD_DbgLog("Main Sensor wait...\n");
		for(;;) //Sensor wait
		{
			if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET) break; //Sensor OFF
			if(Rst_Flag == pba_true) break;
		}
*/

//============================================================================================================
		else if(g_type == OUT) //out mode
		{
			out_sum_count = out_ok_count + out_ng_count;
			total_sum_count = in_ok_count + in_ng_count + out_ok_count + out_ng_count;

			if(Out_state == pba_true) //Right ok
			{
				lock_count = 0;

				Beep(2,50);

				HAL_GPIO_WritePin(OK_LED_GPIO_Port, OK_LED_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(RLED_OK_GPIO_Port, RLED_OK_Pin, GPIO_PIN_RESET);
				Result_Display(pba_none, pba_true);
				out_ok_count++;
				total_ok_count = in_ok_count + out_ok_count;

				if(g_mark == MARK_ON)
				{
					RMARK_ON;
					HAL_Delay(300);
					RMARK_OFF;
				}
			}
			else
			{
				HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(NG_LED_GPIO_Port, NG_LED_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(RLED_NG_GPIO_Port, RLED_NG_Pin, GPIO_PIN_RESET);

				Result_Display(pba_none, pba_false);
				out_ng_count++;
				total_ng_count = in_ng_count + out_ng_count;

				if(g_error == DEBUG_ON) if(lock_count < 5) lock_count++;
			}
		}
		else if(g_type == ALL) //in+out mode
		{
			in_sum_count = in_ok_count + in_ng_count;
			out_sum_count = out_ok_count + out_ng_count;
			total_sum_count = in_ok_count + in_ng_count + out_ok_count + out_ng_count;

			if(In_state == pba_false && Out_state == pba_false) //all fail
			{
				HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(NG_LED_GPIO_Port, NG_LED_Pin, GPIO_PIN_RESET);

				HAL_GPIO_WritePin(LLED_NG_GPIO_Port, LLED_NG_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(RLED_NG_GPIO_Port, RLED_NG_Pin, GPIO_PIN_RESET);
				Result_Display(pba_false, pba_false);

				in_ng_count++;
				out_ng_count++;
				total_ng_count = in_ng_count + out_ng_count;

				if(g_error == DEBUG_ON) if(lock_count < 5) lock_count++;
			}
			else if(In_state == pba_false && Out_state == pba_true) //Right ok
			{
				HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(NG_LED_GPIO_Port, NG_LED_Pin, GPIO_PIN_RESET);

				HAL_GPIO_WritePin(LLED_NG_GPIO_Port, LLED_NG_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(RLED_OK_GPIO_Port, RLED_OK_Pin, GPIO_PIN_RESET);

				Result_Display(pba_false, pba_true);
				in_ng_count++;
				out_ok_count++;
				total_ng_count = in_ng_count + out_ng_count;
				total_ok_count = in_ok_count + out_ok_count;


				if(g_error == DEBUG_ON) if(lock_count < 5) lock_count++;

				if(g_mark == MARK_ON)
				{
					RMARK_ON;
					HAL_Delay(300);
					RMARK_OFF;
				}
			}
			else if(In_state == pba_true && Out_state == pba_false) //Left ok
			{
				HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(NG_LED_GPIO_Port, NG_LED_Pin, GPIO_PIN_RESET);

				HAL_GPIO_WritePin(LLED_OK_GPIO_Port, LLED_OK_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(RLED_NG_GPIO_Port, RLED_NG_Pin, GPIO_PIN_RESET);

				Result_Display(pba_true, pba_false);
				in_ok_count++;
				out_ng_count++;
				total_ok_count = in_ok_count + out_ok_count;
				total_ng_count = in_ng_count + out_ng_count;

				if(g_error == DEBUG_ON) if(lock_count < 5) lock_count++;

				if(g_mark == MARK_ON)
				{
					LMARK_ON;
					HAL_Delay(300);
					LMARK_OFF;
				}
			}
			else if(In_state == pba_true && Out_state == pba_true) //Left, Right ok
			{
				Beep(2,50);

				if(g_error == DEBUG_ON) lock_count = 0;

				HAL_GPIO_WritePin(OK_LED_GPIO_Port, OK_LED_Pin, GPIO_PIN_RESET);

				HAL_GPIO_WritePin(LLED_OK_GPIO_Port, LLED_OK_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(RLED_OK_GPIO_Port, RLED_OK_Pin, GPIO_PIN_RESET);

				Result_Display(pba_true, pba_true);
				in_ok_count++;
				out_ok_count++;
				total_ok_count = in_ok_count + out_ok_count;

				if(g_mark == MARK_ON)
				{
					LMARK_ON;
					RMARK_ON;
					HAL_Delay(300);
					LMARK_OFF;
					RMARK_OFF;
				}
			}
		}

		g_rbuf[0] = (uint8_t)(in_ok_count & 0x000000FF);
		g_rbuf[1] = (uint8_t)((in_ok_count & 0x0000FF00) >> 8);
		g_rbuf[2] = (uint8_t)((in_ok_count & 0x00FF0000) >> 16);
		g_rbuf[3] = (uint8_t)((in_ok_count & 0xFF000000) >> 24);

		eep_256_burst_write(IN_OK_ADDRESS, g_rbuf, 4);
		sprintf((char*)g_buff, "t16.txt=\"%5ld\"", in_ok_count);
		TFT_Serial(g_buff);

		g_rbuf[0] = (uint8_t)(in_ng_count & 0x000000FF);
		g_rbuf[1] = (uint8_t)((in_ng_count & 0x0000FF00) >> 8);
		g_rbuf[2] = (uint8_t)((in_ng_count & 0x00FF0000) >> 16);
		g_rbuf[3] = (uint8_t)((in_ng_count & 0xFF000000) >> 24);

		eep_256_burst_write(IN_NG_ADDRESS, g_rbuf, 4);
		sprintf((char*)g_buff, "t24.txt=\"%5ld\"", in_ng_count);
		TFT_Serial(g_buff);

		in_sum_count = in_ok_count + in_ng_count;
		sprintf((char*)g_buff, "t20.txt=\"%5ld\"", in_sum_count);
		TFT_Serial(g_buff);

		g_rbuf[0] = (uint8_t)(out_ok_count & 0x000000FF);
		g_rbuf[1] = (uint8_t)((out_ok_count & 0x0000FF00) >> 8);
		g_rbuf[2] = (uint8_t)((out_ok_count & 0x00FF0000) >> 16);
		g_rbuf[3] = (uint8_t)((out_ok_count & 0xFF000000) >> 24);

		eep_256_burst_write(OUT_OK_ADDRESS, g_rbuf, 4);
		sprintf((char*)g_buff, "t17.txt=\"%5ld\"", out_ok_count);
		TFT_Serial(g_buff);

		g_rbuf[0] = (uint8_t)(out_ng_count & 0x000000FF);
		g_rbuf[1] = (uint8_t)((out_ng_count & 0x0000FF00) >> 8);
		g_rbuf[2] = (uint8_t)((out_ng_count & 0x00FF0000) >> 16);
		g_rbuf[3] = (uint8_t)((out_ng_count & 0xFF000000) >> 24);

		eep_256_burst_write(OUT_NG_ADDRESS, g_rbuf, 4);
		sprintf((char*)g_buff, "t26.txt=\"%5ld\"", out_ng_count);
		TFT_Serial(g_buff);

		out_sum_count = out_ok_count + out_ng_count;
		sprintf((char*)g_buff, "t25.txt=\"%5ld\"", out_sum_count);
		TFT_Serial(g_buff);

		total_ok_count = in_ok_count + out_ok_count;
		sprintf((char*)g_buff, "t18.txt=\"%5ld\"", total_ok_count);
		TFT_Serial(g_buff);

		total_ng_count = in_ng_count + out_ng_count;
		sprintf((char*)g_buff, "t28.txt=\"%5ld\"", total_ng_count);
		TFT_Serial(g_buff);

		total_sum_count = in_ok_count + in_ng_count + out_ok_count + out_ng_count;
		sprintf((char*)g_buff, "t29.txt=\"%5ld\"", total_sum_count);
		TFT_Serial(g_buff);

		MAIN_OFF;
		LCD_UsrLog("Main Cylinder OFF\n");
		LCD_DbgLog("Main Sensor wait...\n");
		for(;;) //Sensor wait
		{
			if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET) break; //Sensor OFF
			if(Rst_Flag == pba_true) break;
		}
//		VAC_OFF;
//		VAC1_OFF;
		Vaccum_Push_Set();

		sprintf((char*)g_buff, "lock count : %d\r\n", lock_count);
		Debug_serial((char*)g_buff);

		if(lock_count == 5 && g_error == DEBUG_ON)
		{
			g_lock_on = LOCK_ON;

			g_rbuf[0] = (uint8_t)(g_lock_on & 0x000000FF);
			g_rbuf[1] = (uint8_t)((g_lock_on & 0x0000FF00) >> 8);
			g_rbuf[2] = (uint8_t)((g_lock_on & 0x00FF0000) >> 16);
			g_rbuf[3] = (uint8_t)((g_lock_on & 0xFF000000) >> 24);

			eep_256_burst_write(LOCK_ON_ADDRESS, g_rbuf, 4);

			lock_count = 0;
		}
	}
}

void Rst_Task(void)
{
	if(Rst_Flag == pba_true)
	{
		Rst_Flag = pba_false;
		Start_Flag = pba_false;
		Result_Flag = pba_false;
		Test_Result_Flag = pba_false;

		gkey_count = 0;
		gmode_count = 0;
		count1 = 0;
		count2 = 0;
		count3 = 0;
		Vaccum_Push = 0;
		Push_Flag = pba_false;
		if(mode_state == pba_false)
		{
			while(HAL_GPIO_ReadPin(RESET_KEY_GPIO_Port, RESET_KEY_Pin) == GPIO_PIN_RESET);

			MAIN_OFF;
			VAC_OFF;
			VAC1_OFF;
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

			HAL_GPIO_WritePin(FET1_GPIO_Port, FET1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(FET2_GPIO_Port, FET2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(FET3_GPIO_Port, FET3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(FET4_GPIO_Port, FET4_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(FET5_GPIO_Port, FET5_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(FET6_GPIO_Port, FET6_Pin, GPIO_PIN_SET);

			//DFF_Floating_Init();

			LCD_DbgLog("User Press Reset Key...\n");

			Vaccum_Push_Set();


			Beep(1,50);
		}

		if((g_test == DEBUG_ON) && (Debug_Flag == pba_false))
		{
			TFT_Serial((uint8_t*)"page 4");			//¸¶½ºÅÍ ½Ă·á °ËÁơ ½Ă LCD È®ÀÎÀ¸·Î ÆäÀ̀Áö º¯°æ

			if(ok_count == 1) TFT_Serial((uint8_t*)"t4.txt=\"O\""); //OK
			if(open_count == 1) TFT_Serial((uint8_t*)"t5.txt=\"O\""); //OPEN
			if(short_count == 1) TFT_Serial((uint8_t*)"t6.txt=\"O\""); //SHORT
		}

	}
}

void Open_Test(void) //»ç¿ë ±ƯÁö
{
	void(*Func)(void) = NULL;
	int channel = 1;
	float res;

	HAL_GPIO_WritePin(FET1_GPIO_Port, FET1_Pin, GPIO_PIN_RESET); //20 ohm

	for(int i=0; i<40; i++)
	{
		if(i==0) Func = DFF1;
		else if(i==1) Func = DFF2;
		else if(i==2) Func = DFF3;
		else if(i==3) Func = DFF4;
		else if(i==4) Func = DFF5;
		else if(i==5) Func = DFF6;
		else if(i==6) Func = DFF7;
		else if(i==7) Func = DFF8;
		else if(i==8) Func = DFF9;
		else if(i==9) Func = DFF10;
		else if(i==10) Func = DFF11;
		else if(i==11) Func = DFF12;
		else if(i==12) Func = DFF13;
		else if(i==13) Func = DFF14;
		else if(i==14) Func = DFF15;
		else if(i==15) Func = DFF16;
		else if(i==16) Func = DFF17;
		else if(i==17) Func = DFF18;
		else if(i==18) Func = DFF19;
		else if(i==19) Func = DFF20;
		else if(i==20) Func = DFF21;
		else if(i==21) Func = DFF22;
		else if(i==22) Func = DFF23;
		else if(i==23) Func = DFF24;
		else if(i==24) Func = DFF25;
		else if(i==25) Func = DFF26;
		else if(i==26) Func = DFF27;
		else if(i==27) Func = DFF28;
		else if(i==28) Func = DFF29;
		else if(i==29) Func = DFF30;
		else if(i==30) Func = DFF31;
		else if(i==31) Func = DFF32;
		else if(i==32) Func = DFF33;
		else if(i==33) Func = DFF34;
		else if(i==34) Func = DFF35;
		else if(i==35) Func = DFF36;
		else if(i==36) Func = DFF37;
		else if(i==37) Func = DFF38;
		else if(i==38) Func = DFF39;
		else if(i==39) Func = DFF40;

		for(int num = 0; num < 8; num++)
		{
			DFF_Data(0x01 << num);
			Func();

			res = OS_Function(OPEN);

			if((res < 400) && (res > 300))
			{
				LCD_OkLog("CH[%3d] : %.2f[ohm]\n", channel, res);
			}
			else LCD_ErrLog("CH[%3d] : %.2f[ohm]\n", channel, res);

			DFF_Data(0x00);
			Func();

			channel++;
		}

		if((i+1) % 5 == 0)
		{
			while(HAL_GPIO_ReadPin(MODE_KEY_GPIO_Port, MODE_KEY_Pin) == GPIO_PIN_SET);
			while(HAL_GPIO_ReadPin(MODE_KEY_GPIO_Port, MODE_KEY_Pin) == GPIO_PIN_RESET);
			Beep(1,50);
		}
	}

	HAL_GPIO_WritePin(FET1_GPIO_Port, FET1_Pin, GPIO_PIN_SET);
}

void _delay_ms(uint32_t ms_delay)
{
	uint32_t tick = HAL_GetTick();

	for(;;)
	{
		if(Rst_Flag == pba_true) break;
		if(HAL_GetTick() - tick == ms_delay)
		{
			break;
		}
	}
}

void Short_Test(void) //»ç¿ë ±ƯÁö
{
	void(*Func)(void) = NULL;
	int channel = 1;
	float res;

//	HAL_GPIO_WritePin(FET6_GPIO_Port, FET6_Pin, GPIO_PIN_RESET); //200K
	Fet_spec(1,FET_ON);
	HAL_Delay(1);

	for(int i=0; i<40; i++)
	{
		if(i==0) Func = DFF1;
		else if(i==1) Func = DFF2;
		else if(i==2) Func = DFF3;
		else if(i==3) Func = DFF4;
		else if(i==4) Func = DFF5;
		else if(i==5) Func = DFF6;
		else if(i==6) Func = DFF7;
		else if(i==7) Func = DFF8;
		else if(i==8) Func = DFF9;
		else if(i==9) Func = DFF10;
		else if(i==10) Func = DFF11;
		else if(i==11) Func = DFF12;
		else if(i==12) Func = DFF13;
		else if(i==13) Func = DFF14;
		else if(i==14) Func = DFF15;
		else if(i==15) Func = DFF16;
		else if(i==16) Func = DFF17;
		else if(i==17) Func = DFF18;
		else if(i==18) Func = DFF19;
		else if(i==19) Func = DFF20;
		else if(i==20) Func = DFF21;
		else if(i==21) Func = DFF22;
		else if(i==22) Func = DFF23;
		else if(i==23) Func = DFF24;
		else if(i==24) Func = DFF25;
		else if(i==25) Func = DFF26;
		else if(i==26) Func = DFF27;
		else if(i==27) Func = DFF28;
		else if(i==28) Func = DFF29;
		else if(i==29) Func = DFF30;
		else if(i==30) Func = DFF31;
		else if(i==31) Func = DFF32;
		else if(i==32) Func = DFF33;
		else if(i==33) Func = DFF34;
		else if(i==34) Func = DFF35;
		else if(i==35) Func = DFF36;
		else if(i==36) Func = DFF37;
		else if(i==37) Func = DFF38;
		else if(i==38) Func = DFF39;
		else if(i==39) Func = DFF40;

		for(int num = 0; num < 8; num++)
		{
			DFF_Data(0x01 << num);
			Func();

			res = OS_Function(Dbg);

			if(res > SHORT_SPEC)
			{
				LCD_OkLog("CH[%3d] : %.2f[ohm]\n", channel, res);
			}
			else LCD_ErrLog("CH[%3d] : %.2f[ohm]\n", channel, res);

			DFF_Data(0x00);
			Func();

			channel++;
		}

		if((i+1) % 5 == 0)
		{
			while(HAL_GPIO_ReadPin(MODE_KEY_GPIO_Port, MODE_KEY_Pin) == GPIO_PIN_SET);
			while(HAL_GPIO_ReadPin(MODE_KEY_GPIO_Port, MODE_KEY_Pin) == GPIO_PIN_RESET);
			Beep(1,50);
		}
	}

	HAL_GPIO_WritePin(FET2_GPIO_Port, FET2_Pin, GPIO_PIN_SET);
}

void Test_mode_Result(void)
{
	if(Test_Result_Flag == pba_true)
	{
		Test_Result_Flag = pba_false;

		TFT_Serial((uint8_t*)"page 4");
		if(g_type == IN) TFT_Serial((uint8_t*)"t0.txt=\"IN SAMPLE TEST MODE\"");
		else if(g_type == OUT) TFT_Serial((uint8_t*)"t0.txt=\"OUT SAMPLE TEST MODE\"");
		else if(g_type == ALL) TFT_Serial((uint8_t*)"t0.txt=\"BOTH SAMPLE TEST MODE\"");

		pba_bool In_state = In_ostate & In_sstate;
		pba_bool Out_state = Out_ostate & Out_sstate;

//		static int ok_count = 0, open_count = 0, short_count = 0;	//Àü¿ªº¯¼ö·Î º¯°æ. (Reset½Ă Ä«¿îÆ® °ª È®ÀÎ)

		if(g_type == IN) //LEFT
		{
			if(In_ostate == pba_true && In_sstate == pba_true)
			{
			//	TFT_Serial((uint8_t*)"t4.txt=\"O\""); //OK
				if(ok_count == 0) ok_count++;
			}
			else if(In_ostate == pba_false && In_sstate == pba_true)
			{
			//	TFT_Serial((uint8_t*)"t5.txt=\"O\""); //OPEN
				if(open_count == 0) open_count++;
			}
			else if(In_ostate == pba_true && In_sstate == pba_false)
			{
			//	TFT_Serial((uint8_t*)"t6.txt=\"O\""); //SHORT
				if(short_count == 0) short_count++;
			}
		}
		else if(g_type == OUT) //RIGHT
		{
			if(Out_ostate == pba_true && Out_sstate == pba_true)
			{
			//	TFT_Serial((uint8_t*)"t4.txt=\"O\""); //OK
				if(ok_count == 0) ok_count++;
			}
			else if(Out_ostate == pba_false && Out_sstate == pba_true)
			{
			//	TFT_Serial((uint8_t*)"t5.txt=\"O\""); //OPEN
				if(open_count == 0) open_count++;
			}
			else if(Out_ostate == pba_true && Out_sstate == pba_false)
			{
			//	TFT_Serial((uint8_t*)"t6.txt=\"O\""); //SHORT
				if(short_count == 0) short_count++;
			}
		}
		else if(g_type == ALL) //LEFT+RIGHT
		{
			if(In_state == pba_true && Out_state == pba_true) //ALL OK
			{
			//	TFT_Serial((uint8_t*)"t4.txt=\"O\""); //OK
				if(ok_count == 0) ok_count++;
			}
			else
			{
				if(In_ostate == pba_false && In_sstate == pba_true && Out_ostate == pba_false && Out_sstate == pba_true) //ALL OPEN
				{
				//	TFT_Serial((uint8_t*)"t5.txt=\"O\""); //IN&OUT ALL OPEN
					if(open_count == 0) open_count++;
				}
				else if(In_ostate == pba_true && In_sstate == pba_false && Out_ostate == pba_true && Out_sstate == pba_false) //ALL SHORT
				{
				//	TFT_Serial((uint8_t*)"t6.txt=\"O\""); //IN&OUT ALL SHORT
					if(short_count == 0) short_count++;
				}
			}
		}

		if(ok_count == 1) TFT_Serial((uint8_t*)"t4.txt=\"O\""); //OK
		if(open_count == 1) TFT_Serial((uint8_t*)"t5.txt=\"O\""); //OPEN
		if(short_count == 1) TFT_Serial((uint8_t*)"t6.txt=\"O\""); //SHORT

		sprintf((char*)g_buff, "count display : %d, %d, %d\r\n", ok_count, open_count, short_count);
		Debug_serial((char*)g_buff);

		MAIN_OFF;
		VAC_OFF;
		VAC1_OFF;
		Vaccum_Push_Set();

		if(ok_count == 1 && open_count == 1 && short_count == 1)
		{
			ok_count = 0;
			open_count = 0;
			short_count = 0;

			Debug_Flag = pba_true;
			TFT_Serial((uint8_t*)"page 1");
			BSP_LCD_Init();
			Banner_Display();
			Count_Display();
			Beep(2,50);
		}
	}

}


void Error_Stop(void)
{
	if(g_lock_on == LOCK_ON)
	{
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET); //Buzzer OFF

		Debug_serial("LOCK ON\r\n");
		TFT_Serial((uint8_t*)"page 0");
		TFT_Serial((uint8_t*)"xstr 0,90,500,80,5,RED,WHITE,0,0,1,\"ERROR STOP\"");

		while(1)
		{
			if((HAL_GPIO_ReadPin(MODE_KEY_GPIO_Port, MODE_KEY_Pin) == GPIO_PIN_RESET) && (HAL_GPIO_ReadPin(LEFT_KEY_GPIO_Port, LEFT_KEY_Pin) == GPIO_PIN_RESET))
			{
				HAL_Delay(10);

				while((HAL_GPIO_ReadPin(MODE_KEY_GPIO_Port, MODE_KEY_Pin) == GPIO_PIN_RESET) || (HAL_GPIO_ReadPin(LEFT_KEY_GPIO_Port, LEFT_KEY_Pin) == GPIO_PIN_RESET));

				Beep(1,50);

				g_lock_on = LOCK_OFF;

				g_rbuf[0] = (uint8_t)(g_lock_on & 0x000000FF);
				g_rbuf[1] = (uint8_t)((g_lock_on & 0x0000FF00) >> 8);
				g_rbuf[2] = (uint8_t)((g_lock_on & 0x00FF0000) >> 16);
				g_rbuf[3] = (uint8_t)((g_lock_on & 0xFF000000) >> 24);

				eep_256_burst_write(LOCK_ON_ADDRESS, g_rbuf, 4);

				break;
			}
		}

		TFT_Serial((uint8_t*)"page 1");
		/*TFT_Serial((uint8_t*)"draw 0,0,335,271,WHITE");
		TFT_Serial((uint8_t*)"draw 335,0,479,271,WHITE");
		TFT_Serial((uint8_t*)"line 0,20,479,20,WHITE");
		TFT_Serial((uint8_t*)"line 0,40,479,40,WHITE");
		TFT_Serial((uint8_t*)"line 0,60,479,60,WHITE");
		TFT_Serial((uint8_t*)"line 336,126,479,126,WHITE");
		TFT_Serial((uint8_t*)"line 336,191,479,191,WHITE");
		TFT_Serial((uint8_t*)"xstr 380,64,95,60,2,WHITE,BLACK,1,1,1,\"-\"");
		TFT_Serial((uint8_t*)"xstr 380,129,95,60,2,WHITE,BLACK,1,1,1,\"-\"");
*/
		Banner_Display();
		Count_Display();

		LCD_DbgLog("User Break Lock Mode\n");
	}
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
