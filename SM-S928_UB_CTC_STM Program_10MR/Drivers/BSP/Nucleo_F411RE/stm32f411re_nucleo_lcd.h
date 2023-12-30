/*
 * stm32f411re_nucleo_lcd.h
 *
 *  Created on: 2019. 3. 6.
 *      Author: SEO DONG JIN
 */

#ifndef BSP_NUCLEO_F411RE_STM32F411RE_NUCLEO_LCD_H_
#define BSP_NUCLEO_F411RE_STM32F411RE_NUCLEO_LCD_H_

#include "uart_tftlcd\uart_tftlcd.h"

typedef enum
{
  LCD_OK = 0,
  LCD_ERROR = 1,
  LCD_TIMEOUT = 2
}LCD_StatusTypeDef;

typedef enum
{
  CENTER_MODE             = 0x01,    /* center mode */
  RIGHT_MODE              = 0x02,    /* right mode  */
  LEFT_MODE               = 0x03,    /* left mode   */
}Text_AlignModeTypdef;

typedef struct
{
  uint32_t  TextColor;
  uint32_t  BackColor;
  uint16_t Width;
  uint16_t Height;
}LCD_DrawPropTypeDef;

typedef enum
{
	WHITE,
	RED,
	GREEN,
	BLUE,
	YELLOW,
	BLACK
}COLOR;

#define MAX_LAYER_NUMBER 1

//void TFT_Serial(uint8_t* string);

uint8_t BSP_LCD_Init(void);
uint16_t BSP_LCD_GetTextColor(void);
uint16_t BSP_LCD_GetBackColor(void);
void BSP_LCD_SetTextColor(uint32_t Color);
void BSP_LCD_SetBackColor(uint32_t Color);
uint32_t BSP_LCD_GetXSize(void);
uint32_t BSP_LCD_GetYSize(void);
void BSP_LCD_DisplayStringAt(uint16_t X, uint16_t Y, uint8_t *pText, Text_AlignModeTypdef mode);
void BSP_LCD_DisplayStringAtLine(uint16_t Line, uint8_t *ptr);
void BSP_LCD_ClearStringLine(int Line);
void BSP_LCD_Clear(uint32_t color);

#endif /* BSP_NUCLEO_F411RE_STM32F411RE_NUCLEO_LCD_H_ */
