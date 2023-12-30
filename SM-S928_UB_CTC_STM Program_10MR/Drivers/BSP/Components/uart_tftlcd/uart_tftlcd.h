/*
 * uart_tftlcd.h
 *
 *  Created on: 2019. 3. 6.
 *      Author: SEO DONG JIN
 */

#ifndef BSP_COMPONENTS_UART_TFTLCD_UART_TFTLCD_H_
#define BSP_COMPONENTS_UART_TFTLCD_UART_TFTLCD_H_

#include "../Common/lcd.h"

//void TFT_Serial(char *string);
void uart_tftlcd_Init(void);
uint16_t uart_tftlcd_GetLcdPixelWidth(void);
uint16_t uart_tftlcd_GetLcdPixelHeight(void);

extern LCD_DrvTypeDef   uart_tftlcd_drv;

#define  LCD_PIXEL_WIDTH    ((uint16_t)480)
#define  LCD_PIXEL_HEIGHT   ((uint16_t)270)

#endif /* BSP_COMPONENTS_UART_TFTLCD_UART_TFTLCD_H_ */
