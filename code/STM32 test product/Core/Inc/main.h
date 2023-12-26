
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "Nextion.h"
#include "delay.h"
#include "sofware_i2c.h"
#include "MCP23017.h"
#include "HC74.h"

#define LCD_ON              						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET)
#define LCD_OFF              						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET)

#define VAN_OFF                          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET)
#define VAN_ON                          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET)

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
