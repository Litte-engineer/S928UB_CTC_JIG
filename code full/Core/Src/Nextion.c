#include "Nextion.h"

void Write_String(uint8_t line, char* color, char* str);
void (*tft_serial)(uint8_t, char*, char*) = &Write_String;

// xstr 5,100,50,20,0,GREEN,BLACK,1,1,0,"hello"
uint8_t Cmd_end[3] = {0xff,0xff,0xff};
extern char* lcd_data[MAX_CAPCHE];
extern char* lcd_text_color[MAX_CAPCHE];
extern uint8_t lcd_count_line;
extern uint8_t lcd_status_touch;
extern uint8_t lcd_old_line_mumber;
extern uint8_t lcd_reset_line_up;
extern char* test_result[20];

uint8_t lcd_mumber_line;
uint8_t lcd_current_line_up;
uint8_t lcd_current_line_down;
uint8_t lcd_mumber_line_down;
uint8_t lcd_count;
uint32_t lcd_time;

void end_transmit(void)
{
	HAL_UART_Transmit(&UART_NEXTION, Cmd_end, 3, 100);
	HAL_Delay(10);
}

/*************** chuyen page **********************/
void Nextion_page(uint8_t page) // 70 61 67 65 20 30 ff ff ff
{
	char buff[10];
	int len = sprintf(buff, "page %d", page);
	HAL_UART_Transmit(&UART_NEXTION, (uint8_t*)buff, len, 100);
	end_transmit();
	HAL_Delay(70);
}
/******* hien thi text *******************/
void Nextion_Send_String(char *ID, char *string)
{
    char buf[50];
    int len = sprintf(buf,"%s.txt=\"%s\"", ID, string);
    HAL_UART_Transmit(&UART_NEXTION, (uint8_t*)buf, len, 100);
		end_transmit();
}

/********** ham hien thi so 32 bit ********/
void Nextion_Send_Float(char *obj, int32_t num)
{
    char buffer[50];
    int len = sprintf(buffer,"%s.val=%d", obj, num);
    HAL_UART_Transmit(&UART_NEXTION, (uint8_t*)buffer, len, 100);
		end_transmit();
}
/********** ham hien thi float *************/
void Nextion_Float_Point(char *obj, uint8_t num)
{
    char buffer[50];
    int len = sprintf(buffer,"%s.vvs1=%d", obj, num);
    HAL_UART_Transmit(&UART_NEXTION, (uint8_t*)buffer, len, 100);
		end_transmit();
}
/****** ham cai dat mau sac cho o id ****/
void Nextion_SetColor(char* ID, uint32_t color)
{
		char buffer[20];
		int len =  sprintf(buffer,"%s.pco=%d", ID, color);
		HAL_UART_Transmit(&UART_NEXTION, (uint8_t*)buffer, len, 100);
		end_transmit();
}

/****** hien thi cac du lieu tren dong ****/
void Write_String(uint8_t line, char* color, char* str)
{
		uint8_t line_y;
		switch (line)
		{
			case 0 :
				line_y = 61; // toa do dong 0
			break;
			case 1 :
				line_y = 77; // toa do dong 1
			break;
			case 2 :
				line_y = 93; // toa do dong 2
			break;
			case 3 :
				line_y = 109; // toa do dong 3
			break;
			case 4 :
				line_y = 125; // toa do dong 4
			break;
			case 5 :
				line_y = 141; // toa do dong 5
			break;
			case 6 :
				line_y = 157; // toa do dong 6
			break;
			case 7 :
				line_y = 173; // toa do dong 7
			break;
			case 8 :
				line_y = 189; // toa do dong 8
			break;
			case 9 :
				line_y = 205; // toa do dong 9
			break;
			case 10:
				line_y = 221; // toa do dong 10
			break;
			case 11 :
				line_y = 237; // toa do dong 11
			break;
			case 12:
				line_y = 253; // toa do dong 12
			break;	
		}
		char buffer[100];
	  uint16_t length = sprintf((char*)buffer, "xstr 5,%d,480,16,4,%s,BLACK,0,1,1,\"%s\"",line_y, color, str);
		HAL_UART_Transmit(&UART_NEXTION, (uint8_t*)buffer, length, 0xFFFF); 
		end_transmit();	
}
/******* ham hien thi ket qua test **********/
void lcd_result(uint8_t result, uint8_t in_out)
{
	if(in_out == IN)
	{
		if(result == NO_TEST)
		{
			char* str = "-";
			uint8_t buffer[50];
			uint16_t len = sprintf((char*)buffer, "xstr 420,70,480,40,11,WHITE,BLACK,0,1,1,\"%s\"", str);
			HAL_UART_Transmit(&UART_NEXTION, buffer, len, 100);
			end_transmit();
		}
		else if(result == OK)
		{
			char* str = "OK";
			uint8_t buffer[50];
			uint16_t len = sprintf((char*)buffer, "xstr 400,75,480,40,11,GREEN,BLACK,0,1,1,\"%s\"", str);
			HAL_UART_Transmit(&UART_NEXTION, buffer, len, 100);
			end_transmit();
		}
		else if(result == NG)
		{
			char* str = "NG";
			uint8_t buffer[50];
			uint16_t len = sprintf((char*)buffer, "xstr 400,75,480,40,11,RED,BLACK,0,1,1,\"%s\"", str);
			HAL_UART_Transmit(&UART_NEXTION, buffer, len, 100);
			end_transmit();
		}	
	}
	else if(in_out == OUT)
	{
		if(result == NO_TEST )
		{
			char* str = "-";
			uint8_t buffer[50];
			uint16_t len = sprintf((char*)buffer, "xstr 420,135,480,40,11,WHITE,BLACK,0,1,1,\"%s\"", str);
			HAL_UART_Transmit(&UART_NEXTION, buffer, len, 100);
			end_transmit();
		}
		else if(result == OK)
		{
			char* str = "OK";
			uint8_t buffer[50];
			uint16_t len = sprintf((char*)buffer, "xstr 400,140,480,40,11,GREEN,BLACK,0,1,1,\"%s\"", str);
			HAL_UART_Transmit(&UART_NEXTION, buffer, len, 100);
			end_transmit();
		}
		else if(result == NG)
		{
			char* str = "NG";
			uint8_t buffer[50];
			uint16_t len = sprintf((char*)buffer, "xstr 400,140,480,40,11,RED,BLACK,0,1,1,\"%s\"", str);
			HAL_UART_Transmit(&UART_NEXTION, buffer, len, 100);
			end_transmit();
		}	
	}
}

/********* ham hien thi so tren o text lcd ********/
void lcd_UintToChar(char *ID,uint16_t mumber)
{
	char* buffer[50];
	uint16_t len = sprintf((char*)buffer,"%d", mumber);
	char *buff[len];
	for(uint8_t i = 0; i < len; i++)
	{
		buff[i] = buffer[i];
	}
	char str[50];
	int len1 = sprintf(str,"%s.txt=\"%s\"", ID, (char*)buff);
  HAL_UART_Transmit(&UART_NEXTION, (uint8_t*)str, len1, 100);
	end_transmit();
}
/************* ham update display ****************/
void lcd_update_display(uint8_t *line)
{
	if( lcd_count_line < MAX_CAPCHE)
	{
		for(uint8_t i = 0; i < MUMBER_LINE_LCD + 1; i ++)
		{
			tft_serial(i, lcd_text_color[*line - MUMBER_LINE_LCD  + i], lcd_data[*line - MUMBER_LINE_LCD  + i ]);
		}
	}
}
/********** dieu chinh len xuong man hinh lcd **********/
void up_down_lcd(void)
{
		if(lcd_old_line_mumber != lcd_count_line || lcd_reset_line_up == ON)
		{
			lcd_current_line_up = 0;
			lcd_old_line_mumber = lcd_count_line;
			lcd_reset_line_up = OFF;
		}
		if(lcd_status_touch  ==  LCD_TOUCH_UP)
		{
			if(lcd_current_line_up == 0) lcd_current_line_up = 1;
			if(lcd_current_line_down > 0) lcd_current_line_down--;
			lcd_mumber_line = lcd_count_line - lcd_current_line_up;
			if(lcd_mumber_line > MUMBER_LINE_LCD) lcd_current_line_up ++;
			lcd_update_display(&lcd_mumber_line);		
			lcd_status_touch = LCD_NO_TOUCH;
	  }

		else if(lcd_status_touch == LCD_TOUCH_DOWN)
		{
			if(lcd_current_line_down == 0) lcd_current_line_down = 1;
			if(lcd_current_line_up > 0 ) 
			{
				lcd_current_line_up --;
				lcd_mumber_line_down = lcd_mumber_line + lcd_current_line_down;		
				if( lcd_mumber_line_down < lcd_count_line) lcd_current_line_down ++;	
				lcd_update_display(&lcd_mumber_line_down);	
			}	
			lcd_status_touch = LCD_NO_TOUCH;
		}
}

/******** kiem tra so luong dong ********/
void check_line_lcd(void)
{
	if(lcd_count_line < MAX_CAPCHE - 1) lcd_count_line ++;
	else 
	{
		lcd_reset_line_up = ON;
		lcd_count_line    = MAX_CAPCHE - 1;
		for(int i = 0; i < MAX_CAPCHE - 1; i ++)
		{
			lcd_data[i] 			= lcd_data[i + 1];
			lcd_text_color[i] = lcd_text_color[i + 1];
		}
	}
}
/************ ham them dong moi khi co su kien ***********/
void lcd_add_line(char* str, char* color)
{
	check_line_lcd();
  lcd_data[lcd_count_line] = str;
	lcd_text_color[lcd_count_line] = color;
	lcd_update_display(&lcd_count_line);
}

/******* ham hien thi ket qua test ******/
void result_test_lcd_spen(float res, char* con, char* mode_test)
{
	char* buffer[50];
	char* result;
	char* color;
	
	if(mode_test[0] == 'O') /// test open
	{
		/*** ket qua test khong dat ***/
		if(res >= OPEN_SPEC_ALLOW)  
		{
			result = ERR_C;
			color = RED_C;
		}
		/**** ket qua test OK **/
		else 
		{
			result = OK_C;
			color = GREEN_C;
		}
	}
	else if(mode_test[0] == 'S') // test short
	{
		/**** ket qua test khong dat ***/
		if(res <= SHORT_SPEC_ALLOW) 
		{
			color = RED_C;
			result = ERR_C;
		}
		/**** ket qua test ok ***/
		else 
		{
			result = OK_C;
			color = GREEN_C;
		}
	}
	
	uint8_t len = sprintf((char*)buffer,"[%s]:[%s]:%0.2f[R] %s", result, con, res, mode_test);	
	for(uint8_t i = 0; i < len; i++)
	{
		test_result[i] = buffer[i]; 
	}
	lcd_add_line((char*)buffer, color);
	
}



