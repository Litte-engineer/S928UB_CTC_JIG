/*
 * uart_touch.c
 *
 *  Created on: 2019. 3. 7.
 *      Author: synopex
 */


#include "uart_touch.h"

/* Touch screen driver structure initialization */
TS_DrvTypeDef uart_touch_ts_drv =
{
  NULL,//stmpe811_Init,
  NULL,//stmpe811_ReadID,
  NULL,//stmpe811_Reset,
  NULL,//stmpe811_TS_Start,
  uart_touch_TS_DetectTouch,//stmpe811_TS_DetectTouch,
  uart_touch_TS_GetXY,//stmpe811_TS_GetXY,
  NULL,//stmpe811_TS_EnableIT,
  NULL,//stmpe811_TS_ClearIT,
  NULL,//stmpe811_TS_ITStatus,
  NULL,//stmpe811_TS_DisableIT,
};

extern volatile pba_bool touch_state;

/**
  * @brief  Return if there is touch detected or not.
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval Touch detected state.
  */
uint8_t uart_touch_TS_DetectTouch(uint16_t DeviceAddr)
{
	if(touch_state == pba_true)
	{
		touch_state = pba_false;

		return 1;
	}
	else return 0;
}

extern uint8_t g_touch_buff[9];
/**
  * @brief  Get the touch screen X and Y positions values
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  X: Pointer to X position value
  * @param  Y: Pointer to Y position value
  * @retval None.
  */
void uart_touch_TS_GetXY(uint16_t DeviceAddr, uint16_t *X, uint16_t *Y)
{
	*X = ((uint16_t)g_touch_buff[1] << 8) + g_touch_buff[2];
	*Y = ((uint16_t)g_touch_buff[3] << 8) + g_touch_buff[4];
}

