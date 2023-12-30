/*
 * uart_tftlcd.c
 *
 *  Created on: 2019. 3. 6.
 *      Author: SEO DONG JIN
 */

#include "uart_tftlcd.h"
#include "main.h"

LCD_DrvTypeDef   uart_tftlcd_drv =
{
  uart_tftlcd_Init,
  0,//ili9341_ReadID,
  0,//ili9341_DisplayOn,
  0,//ili9341_DisplayOff,
  0,
  0,
  0,
  0,
  0,
  0,
  uart_tftlcd_GetLcdPixelWidth,
  uart_tftlcd_GetLcdPixelHeight,
  0,
  0,
};

void uart_tftlcd_Init(void)
{
	 HAL_GPIO_WritePin(LCD_POWER_GPIO_Port, LCD_POWER_Pin, GPIO_PIN_SET);
	 HAL_Delay(500);

	TFT_Serial((uint8_t*)"bkcmd=0");
	TFT_Serial((uint8_t*)"page 1");
/*
	TFT_Serial((uint8_t*)"draw 0,0,335,271,WHITE");
	TFT_Serial((uint8_t*)"draw 335,0,479,271,WHITE");
	TFT_Serial((uint8_t*)"line 0,20,479,20,WHITE");
	TFT_Serial((uint8_t*)"line 0,40,479,40,WHITE");
	TFT_Serial((uint8_t*)"line 0,60,479,60,WHITE");
	TFT_Serial((uint8_t*)"line 336,116,479,116,WHITE");
	TFT_Serial((uint8_t*)"line 336,191,479,191,WHITE");
*/

	//TFT_Serial((uint8_t*)"t7.txt=\"CURVE TOUCH\""); //MODEL NAME
	//TFT_Serial((uint8_t*)"t8.txt=\"V0.0.B\""); //FW Version
	//TFT_Serial((uint8_t*)"t9.txt=\"TYPE-I/II\""); //검사기 종류

	/*TFT_Serial((uint8_t*)"xstr 380,62,95,50,2,WHITE,BLACK,1,1,1,\"-\"");
	TFT_Serial((uint8_t*)"xstr 380,121,95,50,2,WHITE,BLACK,1,1,1,\"-\"");
*/
	//TFT_Serial((uint8_t*)"t16.txt=\"65535\""); //ok
	//TFT_Serial((uint8_t*)"t17.txt=\"65535\""); //ng
	//TFT_Serial((uint8_t*)"t18.txt=\"65535\""); //total
}

uint16_t uart_tftlcd_GetLcdPixelWidth(void)
{
  /* Return LCD PIXEL WIDTH */
  return LCD_PIXEL_WIDTH;
}

uint16_t uart_tftlcd_GetLcdPixelHeight(void)
{
  /* Return LCD PIXEL HEIGHT */
  return LCD_PIXEL_HEIGHT;
}
