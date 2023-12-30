/*
 * stm32f411re_nucleo_lcd.c
 *
 *  Created on: 2019. 3. 6.
 *      Author: SEO DONG JIN
 */

#include "stm32f411re_nucleo_lcd.h"
#include "Common\lcd.h"
#include "main.h"

LCD_DrvTypeDef  *LcdDrv;
uint8_t lcd_buffer[300];
LCD_DrawPropTypeDef DrawProp[MAX_LAYER_NUMBER];
uint32_t ActiveLayer = 0;
void Debug_serial(char *buff);

uint8_t BSP_LCD_Init(void)
{
	/* Select the device */
	LcdDrv = &uart_tftlcd_drv;

	/* LCD Init */
	LcdDrv->Init();

	//Init font info
	DrawProp[ActiveLayer].Width = 8;
	DrawProp[ActiveLayer].Height = 16;

	//Init Default Font
	BSP_LCD_SetTextColor(WHITE);
	BSP_LCD_SetBackColor(BLACK);

	return LCD_OK;
}

/**
  * @brief  Gets the LCD text color.
  * @retval Used text color.
  */
uint16_t BSP_LCD_GetTextColor(void)
{
  return DrawProp[ActiveLayer].TextColor;
}

/**
  * @brief  Gets the LCD background color.
  * @retval Used background color
  */
uint16_t BSP_LCD_GetBackColor(void)
{
  return DrawProp[ActiveLayer].BackColor;
}

/**
  * @brief  Sets the LCD text color.
  * @param  Color: Text color code RGB(5-6-5)
  */
void BSP_LCD_SetTextColor(uint32_t Color)
{
  DrawProp[ActiveLayer].TextColor = Color;
}

/**
  * @brief  Sets the LCD background color.
  * @param  Color: Background color code RGB(5-6-5)
  */
void BSP_LCD_SetBackColor(uint32_t Color)
{
  DrawProp[ActiveLayer].BackColor = Color;
}

/**
  * @brief  Gets the LCD X size.
  * @retval Used LCD X size
  */
uint32_t BSP_LCD_GetXSize(void)
{
  return LcdDrv->GetLcdPixelWidth();
}

/**
  * @brief  Gets the LCD Y size.
  * @retval Used LCD Y size
  */
uint32_t BSP_LCD_GetYSize(void)
{
  return LcdDrv->GetLcdPixelHeight();
}

/**
  * @brief  Clears the selected line.
  * @param  Line: Line to be cleared
  *          This parameter can be one of the following values:
  *            @arg  0..9: if the Current fonts is Font16x24
  *            @arg  0..19: if the Current fonts is Font12x12 or Font8x12
  *            @arg  0..29: if the Current fonts is Font8x8
  */
void BSP_LCD_ClearStringLine(int Line)
{
  /* Draw rectangle with background color */
  //BSP_LCD_FillRect(0, (Line * DrawProp[ActiveLayer].pFont->Height), BSP_LCD_GetYSize(), DrawProp[ActiveLayer].pFont->Height);
	sprintf((char*)lcd_buffer, "fill %d,%d,%d,%d,%s", 4, (Line * DrawProp[ActiveLayer].Height) + 62
						,41*DrawProp[ActiveLayer].Width, DrawProp[ActiveLayer].Height, "BLACK");

	TFT_Serial(lcd_buffer);
}

void BSP_LCD_Clear(uint32_t color)
{
	char *text = NULL;

	if(color == WHITE)
	  text = "WHITE";
	else if(color == RED)
	  text = "RED";
	else if(color == GREEN)
	  text = "GREEN";
	else if(color == BLUE)
	  text = "BLUE";
	else if(color == YELLOW)
	  text = "YELLOW";
	else if(color == BLACK)
		  text = "BLACK";
	else
	  text = "WHITE";

	for(int Line = 0; Line < 13; Line++)
	{
		sprintf((char*)lcd_buffer, "fill %d,%d,%d,%d,%s", 4, (Line * DrawProp[ActiveLayer].Height) + 62
									,41*DrawProp[ActiveLayer].Width, DrawProp[ActiveLayer].Height, text);

		TFT_Serial(lcd_buffer);
	}
}
/**
  * @brief  Displays characters on the LCD.
  * @param  Xpos: X position (in pixel)
  * @param  Ypos: Y position (in pixel)
  * @param  Text: Pointer to string to display on LCD
  * @param  Mode: Display mode
  *          This parameter can be one of the following values:
  *            @arg  CENTER_MODE
  *            @arg  RIGHT_MODE
  *            @arg  LEFT_MODE
  */
void BSP_LCD_DisplayStringAt(uint16_t X, uint16_t Y, uint8_t *pText, Text_AlignModeTypdef mode)
{
#if 1
	//User Define(줄/칸 들여쓰기 추가)
	X+=3;
	Y+=64;
	//DrawProp[ActiveLayer].BackColor = BOX_COL;
	//DrawProp[ActiveLayer].TextColor = WHITE;

	DrawProp[ActiveLayer].Width = 8;
	DrawProp[ActiveLayer].Height = 16;
	char ascii_buffer = 0;
	uint8_t xcen = 0;
#endif

	uint16_t refcolumn = 1, i = 0;
	uint32_t size = 0, xsize = 0;
	uint8_t  *ptr = pText;

	/* Get the text size */
	while (*ptr++) size ++ ;

	/* Characters number per line */
	xsize = (BSP_LCD_GetXSize()/DrawProp[ActiveLayer].Width);

	switch (mode)
	{
	  case CENTER_MODE:
		{
		  refcolumn = X + ((xsize - size)* DrawProp[ActiveLayer].Width) / 2;
		  xcen = 1;
		  break;
		}
	  case LEFT_MODE:
		{
		  refcolumn = X;
		  xcen = 0;
		  break;
		}
	  case RIGHT_MODE:
		{
		  refcolumn = X + ((xsize - size)*DrawProp[ActiveLayer].Width);
		  xcen = 2;
		  break;
		}
	  default:
		{
		  refcolumn = X;
		  xcen = 0;
		  break;
		}
	}

	/* Send the string character by character on LCD */
	while ((*pText != 0) & (((BSP_LCD_GetXSize() - (i*DrawProp[ActiveLayer].Width)) & 0xFFFF) >= DrawProp[ActiveLayer].Width))
	{
		/* Display one character on LCD */
		//BSP_LCD_DisplayChar(refcolumn, Y, *pText);
		ascii_buffer = *pText;
		sprintf((char*)lcd_buffer, "xstr %d,%d,%d,%d,4,%s,%s,%d,1,1,\"%c\"", refcolumn, Y
				,DrawProp[ActiveLayer].Width, DrawProp[ActiveLayer].Height, "WHITE", "BLACK", xcen, ascii_buffer); //x,y,w,h,fontid,fontcolor,backcolor,xcen,ycen,1,text

		TFT_Serial(lcd_buffer);
		/* Decrement the column position by 16 */
		refcolumn += DrawProp[ActiveLayer].Width;
		/* Point on the next character */
		pText++;
		i++;
	}
}

/**
  * @brief  Displays a character on the LCD.
  * @param  Line: Line where to display the character shape
  *          This parameter can be one of the following values:
  *            @arg  0..9: if the Current fonts is Font16x24
  *            @arg  0..19: if the Current fonts is Font12x12 or Font8x12
  *            @arg  0..29: if the Current fonts is Font8x8
  * @param  ptr: Pointer to string to display on LCD
  */

void BSP_LCD_DisplayStringAtLine(uint16_t Line, uint8_t *ptr)
{
  //BSP_LCD_DisplayStringAt(0, ((Line) * (DrawProp[ActiveLayer].Height)) , ptr, LEFT_MODE); //느림
  /* Send the string character by character on LCD */
	//uint8_t size = 0;
	//uint8_t *ptr_buffer = NULL;
	char *fore_col = NULL;
	char *back_col = NULL;

	if(DrawProp[ActiveLayer].TextColor == WHITE) fore_col = "WHITE";
	else if(DrawProp[ActiveLayer].TextColor == RED) fore_col = "RED";
	else if(DrawProp[ActiveLayer].TextColor == GREEN) fore_col = "GREEN";
	else if(DrawProp[ActiveLayer].TextColor == BLUE) fore_col = "BLUE";
	else if(DrawProp[ActiveLayer].TextColor == YELLOW) fore_col = "YELLOW";
	else if(DrawProp[ActiveLayer].TextColor == BLACK) fore_col = "BLACK";

	if(DrawProp[ActiveLayer].BackColor == WHITE) back_col = "WHITE";
	else if(DrawProp[ActiveLayer].BackColor == RED) back_col = "RED";
	else if(DrawProp[ActiveLayer].BackColor == GREEN) back_col = "GREEN";
	else if(DrawProp[ActiveLayer].BackColor == BLUE) back_col = "BLUE";
	else if(DrawProp[ActiveLayer].BackColor == YELLOW) back_col = "YELLOW";
	else if(DrawProp[ActiveLayer].BackColor == BLACK) back_col = "BLACK";

	//ptr_buffer = ptr;
/*
	while(*ptr_buffer++)
	{
		size ++;
	}
*/
	sprintf((char*)lcd_buffer, "xstr %d,%d,%d,%d,4,%s,%s,%d,1,1,\"%s\"", 4, (Line * DrawProp[ActiveLayer].Height) + 62
					,41*DrawProp[ActiveLayer].Width, DrawProp[ActiveLayer].Height, fore_col, back_col, 0, ptr);
#if 0
	Debug_serial((char*)lcd_buffer);
	Debug_serial("\r\n");
#endif

	TFT_Serial(lcd_buffer);
}
