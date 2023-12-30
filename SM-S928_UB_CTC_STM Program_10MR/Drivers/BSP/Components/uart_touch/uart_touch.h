/*
 * uart_touch.h
 *
 *  Created on: 2019. 3. 7.
 *      Author: synopex
 */

#ifndef BSP_COMPONENTS_UART_TOUCH_UART_TOUCH_H_
#define BSP_COMPONENTS_UART_TOUCH_UART_TOUCH_H_

#include "Common\ts.h"
#include "main.h"

uint8_t uart_touch_TS_DetectTouch(uint16_t DeviceAddr);
void uart_touch_TS_GetXY(uint16_t DeviceAddr, uint16_t *X, uint16_t *Y);

extern TS_DrvTypeDef uart_touch_ts_drv;

#endif /* BSP_COMPONENTS_UART_TOUCH_UART_TOUCH_H_ */
