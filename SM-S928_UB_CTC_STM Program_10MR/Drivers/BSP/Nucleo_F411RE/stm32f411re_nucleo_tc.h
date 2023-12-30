/*
 * stm32f411re_nucleo_tc.h
 *
 *  Created on: 2019. 3. 7.
 *      Author: synopex
 */

#ifndef BSP_NUCLEO_F411RE_STM32F411RE_NUCLEO_TC_H_
#define BSP_NUCLEO_F411RE_STM32F411RE_NUCLEO_TC_H_

#include "uart_touch\uart_touch.h"
#include "Common\ts.h"

typedef struct
{
  uint16_t TouchDetected;
  uint16_t x;
  uint16_t y;
  uint16_t z;
}TS_StateTypeDef;

typedef enum
{
  TS_OK       = 0x00,
  TS_ERROR    = 0x01,
  TS_TIMEOUT  = 0x02
}TS_StatusTypeDef;

uint8_t BSP_TS_GetState(TS_StateTypeDef *TS_State);
uint8_t BSP_TS_Init(uint16_t xSize, uint16_t ySize);

#endif /* BSP_NUCLEO_F411RE_STM32F411RE_NUCLEO_TC_H_ */
