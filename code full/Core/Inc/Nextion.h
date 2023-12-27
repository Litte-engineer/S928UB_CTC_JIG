#ifndef __NEXTION_H
#define __NEXTION_H

#include "main.h"
#include "stdio.h"

#define RED 63488
#define GREEN 2016
#define YELLOW 65504
#define BLUE 1055
#define BLACK 0
#define WHITE 65535
#define ORANGE 64512
#define BROWN 32768
#define LIGHT_BLUE 34815
#define MAX_CAPCHE 100
#define MUMBER_LINE_LCD 13
#define LCD_TOUCH_UP 1
#define LCD_TOUCH_DOWN 2
#define LCD_NO_TOUCH  0

#define MODEL 													"t7"
#define F_W 														"t8"
#define TYPE 														"t9"
#define DEBUG_MODE 											"t10"
#define GREEN_C 												"GREEN"
#define BLUE_C 													"BLUE"
#define YELLOW_C 												"YELLOW"
#define WHITE_C 												"WHITE"
#define RED_C 													"RED"
#define COUNT_OK 												"t16"
#define COUNT_NG 												"t17"
#define COUNT_TOTAL 										"t18"
#define OK_C                            "Ok"
#define ERR_C                           "Err"
#define UART_NEXTION                    huart1

extern UART_HandleTypeDef  UART_NEXTION;
extern void(*tft_serial)(uint8_t, char*, char*) ;

void Nextion_page(uint8_t page); 
void Nextion_Send_String(char *ID, char *string);
void Nextion_Send_Mumber(char *obj, int32_t num );
void Nextion_Send_Float(char *obj, int32_t num);
void Nextion_Float_Point(char *obj, uint8_t num);
void Nextion_SetColor(char* ID, uint32_t color);
void Write_String(uint8_t line, char*color, char* str);
void lcd_result(uint8_t result, uint8_t in_out);
void lcd_UintToChar(char *ID,uint16_t mumber);
void lcd_update_display(uint8_t *line);
void up_down_lcd(void);
void check_line_lcd(void);
void end_transmit(void);
void lcd_add_line(char* str, char* color);
void result_test_lcd_spen(float res, char* con, char* mode_test);

#endif 
