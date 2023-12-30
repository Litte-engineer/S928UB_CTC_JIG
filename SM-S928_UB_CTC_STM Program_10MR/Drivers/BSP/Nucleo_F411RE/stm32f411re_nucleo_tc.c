/*
 * stm32f411re_nucleo_tc.c
 *
 *  Created on: 2019. 3. 7.
 *      Author: synopex
 */


#include "stm32f411re_nucleo_tc.h"

static TS_DrvTypeDef *ts_driver;

/**
  * @brief  Initializes and configures the touch screen functionalities and
  *         configures all necessary hardware resources (GPIOs, clocks..).
  * @param  xSize: Maximum X size of the TS area on LCD
  * @param  ySize: Maximum Y size of the TS area on LCD
  * @retval TS_OK if all initializations are OK. Other value if error.
  */
uint8_t BSP_TS_Init(uint16_t xSize, uint16_t ySize)
{
	uint8_t status = TS_OK;

	ts_driver = &uart_touch_ts_drv;

	return status;
}

uint8_t BSP_TS_GetState(TS_StateTypeDef *TS_State)
{
  uint16_t x = 0, y = 0;

  TS_State->TouchDetected = ts_driver->DetectTouch(0);

  if(TS_State->TouchDetected)
  {
	  ts_driver->GetXY(0, &x, &y);

	  TS_State->x = x;
	  TS_State->y = y;

	  return TS_OK;
  }

  return TS_ERROR;
}
