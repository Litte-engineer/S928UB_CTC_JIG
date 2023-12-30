/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "stm32f411re_nucleo_lcd.h"
#include "lcd_log.h"
#include "stm32f411re_nucleo_tc.h"

#define SPEC_10M
//#define SPEC_1M

typedef enum _pba_bool
{
	pba_false,
	pba_true,
	pba_none
}pba_bool;

typedef enum _OS_STATE
{
	OPEN = 0,
	RES,
	SHORT,
	Dbg
}OS_STATE;

typedef struct
{
	void(*Func)(void);
	uint8_t pinmap;
}Pair;

typedef struct _OS_Inspect OS_Inspect;
struct _OS_Inspect
{
	char* Group_Name; //name
	uint8_t inspection_num; //inspection num
	uint8_t inspection_kind; //OPEN or SHORT

	Pair *pin;
};
void Auto_Clean(void);
void Vaccum_Push_Set(void);
void TFT_Serial(uint8_t* string);
void Debug_serial(char *buff);
void Touch_Task(void);
void Beep(uint8_t count, uint32_t delay);
void Result_Display(pba_bool result_in, pba_bool result_out);
void Banner_Display(void);
void Count_Display(void);
void DFF_Data(uint8_t data);
int mcp23017_init(uint8_t slave_num);
int eep_256_burst_write(uint32_t addr, uint8_t *MemTarget, uint16_t Size);
int eep_256_burst_read(uint32_t addr, uint8_t *MemTarget, uint16_t Size);
void DFF1(void);
void DFF2(void);
void DFF3(void);
void DFF4(void);
void DFF5(void);
void DFF6(void);
void DFF7(void);
void DFF8(void);
void DFF9(void);
void DFF10(void);
void DFF11(void);
void DFF12(void);
void DFF13(void);
void DFF14(void);
void DFF15(void);
void DFF16(void);
void DFF17(void);
void DFF18(void);
void DFF19(void);
void DFF20(void);
void DFF21(void);
void DFF22(void);
void DFF23(void);
void DFF24(void);
void DFF25(void);
void DFF26(void);
void DFF27(void);
void DFF28(void);
void DFF29(void);
void DFF30(void);
void DFF31(void);
void DFF32(void);
void DFF33(void);
void DFF34(void);
void DFF35(void);
void DFF36(void);
void DFF37(void);
void DFF38(void);
void DFF39(void);
void DFF40(void);
void DFF_GND_Init(void);
void DFF_Floating_Init(void);
void Mode_Task(void);
void Key_input(void);
void Rst_Task(void);
void Start_Task(void);
void Result_Task(void);
void Open_Test(void);
void Short_Test(void);
void _delay_ms(uint32_t ms_delay);
void system_information(void);
void Test_mode_Result(void);
void Error_Stop(void);
//static float OS_Function(OS_STATE value);
//static float R_Function(void);

#define IN 0
#define OUT 1
#define ALL 2
#define MARK_ON 0
#define MARK_OFF 1
#define DEBUG_ON 0
#define DEBUG_OFF 1
#define LOCK_ON 1
#define LOCK_OFF 0
#define KEY_ON 1
#define KEY_OFF 0

#define DFF_DELAY iic_delay_us()
#define PA 0x09
#define PB 0x19
#define SLAVE0 0x40 //8bit address
#define SLAVE1 0x42 //8bit address
#define SLAVE2 0x44 //8bit address
#define SLAVE3 0x46 //8bit address
#define SLAVE4 0x48 //8bit address

#define IN_OK_ADDRESS 0x0000
#define OUT_OK_ADDRESS 0x0004
#define IN_NG_ADDRESS 0x0028
#define OUT_NG_ADDRESS 0x002C

#define TOTAL_ADDRESS 0x0008
#define TYPE_ADDRESS 0x000C
#define MARK_ADDRESS 0x0010
#define DEBUG_ADDRESS 0x0014
#define ERROR_STOP_ADDRESS 0x0018
#define TEST_MODE_ADDRESS 0x001C
#define LOCK_ON_ADDRESS 0x0020
#define CLEAN_ADDRESS 0x0024
#define KEY_INPUT_ADDRESS 0x0030
#define OPEN_SPEC 20
#define SUS_SPEC 10

#ifdef SPEC_10M
	#define SHORT_SPEC 10000000//1000000
#endif
#ifdef SPEC_1M
	#define SHORT_SPEC 1000000
#endif

#define LEFT 10
#define RIGHT 11

extern OS_Inspect IN_SM_S928UB_INSERT[1];
extern OS_Inspect OUT_SM_S928UB_INSERT[1];
extern OS_Inspect IN_SM_S928UB[128];
extern OS_Inspect OUT_SM_S928UB[128];

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CY9_Pin GPIO_PIN_0
#define CY9_GPIO_Port GPIOC
#define CY10_Pin GPIO_PIN_1
#define CY10_GPIO_Port GPIOC
#define CY8_Pin GPIO_PIN_2
#define CY8_GPIO_Port GPIOC
#define CY7_Pin GPIO_PIN_3
#define CY7_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define RLED_OK_Pin GPIO_PIN_6
#define RLED_OK_GPIO_Port GPIOA
#define RLED_NG_Pin GPIO_PIN_7
#define RLED_NG_GPIO_Port GPIOA
#define NG_LED_Pin GPIO_PIN_4
#define NG_LED_GPIO_Port GPIOC
#define SW_SDA_Pin GPIO_PIN_5
#define SW_SDA_GPIO_Port GPIOC
#define LEFT_KEY_Pin GPIO_PIN_1
#define LEFT_KEY_GPIO_Port GPIOB
#define MODE_KEY_Pin GPIO_PIN_2
#define MODE_KEY_GPIO_Port GPIOB
#define FET4_Pin GPIO_PIN_10
#define FET4_GPIO_Port GPIOB
#define RESET_KEY_Pin GPIO_PIN_12
#define RESET_KEY_GPIO_Port GPIOB
#define RESET_KEY_EXTI_IRQn EXTI15_10_IRQn
#define OK_LED_Pin GPIO_PIN_13
#define OK_LED_GPIO_Port GPIOB
#define SKIP_KEY_Pin GPIO_PIN_14
#define SKIP_KEY_GPIO_Port GPIOB
#define RIGHT_KEY_Pin GPIO_PIN_15
#define RIGHT_KEY_GPIO_Port GPIOB
#define SW_SCL_Pin GPIO_PIN_6
#define SW_SCL_GPIO_Port GPIOC
#define FET2_Pin GPIO_PIN_7
#define FET2_GPIO_Port GPIOC
#define LCD_POWER_Pin GPIO_PIN_8
#define LCD_POWER_GPIO_Port GPIOC
#define BUZZER_Pin GPIO_PIN_9
#define BUZZER_GPIO_Port GPIOC
#define FET3_Pin GPIO_PIN_8
#define FET3_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define CY6_Pin GPIO_PIN_15
#define CY6_GPIO_Port GPIOA
#define CY4_Pin GPIO_PIN_10
#define CY4_GPIO_Port GPIOC
#define CY2_Pin GPIO_PIN_11
#define CY2_GPIO_Port GPIOC
#define CY3_Pin GPIO_PIN_12
#define CY3_GPIO_Port GPIOC
#define CY1_Pin GPIO_PIN_2
#define CY1_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define FET5_Pin GPIO_PIN_4
#define FET5_GPIO_Port GPIOB
#define FET6_Pin GPIO_PIN_5
#define FET6_GPIO_Port GPIOB
#define FET1_Pin GPIO_PIN_6
#define FET1_GPIO_Port GPIOB
#define CY5_Pin GPIO_PIN_7
#define CY5_GPIO_Port GPIOB
#define LLED_OK_Pin GPIO_PIN_8
#define LLED_OK_GPIO_Port GPIOB
#define LLED_NG_Pin GPIO_PIN_9
#define LLED_NG_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define LMARK_ON HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET)//J23
#define LMARK_OFF HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET)
#define VAC_ON HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET)//J24
#define VAC_OFF HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET)
#define RMARK_ON HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET)//J25
#define RMARK_OFF HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET)
#define CLEAN_ON HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET)//J26
#define CLEAN_OFF HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET)
#define MAIN_ON HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET)//J27
#define MAIN_OFF HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET)

#define VAC1_ON HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET)//J28
#define VAC1_OFF HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET)
#define CYMARK_ON HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET)//J30
#define CYMARK_OFF HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET)
#define CYMARK_NG_ON HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET)//J32
#define CYMARK_NG_OFF HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET)
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
