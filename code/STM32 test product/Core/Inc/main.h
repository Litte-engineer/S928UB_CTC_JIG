/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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


#define IN 					0
#define OUT 				1
#define IN_OUT 			2
#define ON 					1
#define OFF 				0
#define PAUSE       2
#define PAGE_0 			0
#define PAGE_1 			1
#define PAGE_2 			2
#define PAGE_3 			3
#define PRESS 			0
#define NO_PESS 		1

#define NO_TEST 		0
#define TESTING_1 	3
#define TESTING_2 	4
#define TESTING_3 	5
#define TESTING_4 	6
#define TEST_STOP 	7
#define TEST_START 	2
#define TEST_INIT   1

#define OK 					1
#define NG 					2

#define OPEN_SPEC_ALLOW   10
#define SHORT_SPEC_ALLOW  1000000

#define LCD_ON              						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET)
#define LCD_OFF              						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET)

#define VAN_OFF                          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET)
#define VAN_ON                          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET)

#define SENSOR_READ                     HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)
#define SENSOR_ON                       1
#define SENSOR_OFF                      0

#define FET_20R_ON                      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET)
#define FET_20R_OFF                     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET)
#define FET_200R_ON                     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET)
#define FET_200R_OFF                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET)
#define FET_2K_ON                    	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET)
#define FET_2K_OFF                      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET)
#define FET_20K_ON                      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET)
#define FET_20K_OFF                     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET)
#define FET_200K_ON                     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET)
#define FET_200K_OFF                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET)
#define FET_2M_ON                       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET)
#define FET_2M_OFF                      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET)
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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
