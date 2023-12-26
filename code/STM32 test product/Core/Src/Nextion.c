#include "Nextion.h"

//void Write_String(uint8_t line, char* color, char* str);
//void (*tft_serial)(uint8_t, char*, char*) = &Write_String;

// xstr 5,100,50,20,0,GREEN,BLACK,1,1,0,"hello"
uint8_t Cmd_end[3] = {0xff,0xff,0xff};
/*extern char* lcd_data[MAX_CAPCHE];
extern char* lcd_text_color[MAX_CAPCHE];
extern uint8_t lcd_count_line;
extern uint8_t lcd_status_touch;
extern uint8_t lcd_old_line_mumber;
extern uint8_t lcd_reset_line_up;
extern char* test_result[20];*/

/*uint8_t lcd_mumber_line;
uint8_t lcd_current_line_up;
uint8_t lcd_current_line_down;
uint8_t lcd_mumber_line_down;
uint8_t lcd_count;
uint32_t lcd_time;*/

void end_transmit(void)
{
	HAL_UART_Transmit(&UART_NEXTION, Cmd_end, 3, 100);
	HAL_Delay(5);
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
    HAL_UART_Transmit(&UART_NEXTION, (uint8_t*)buf, len, 0xffff);
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
/*void lcd_update_display(uint8_t *line)
{
	if( lcd_count_line < MAX_CAPCHE)
	{
		for(uint8_t i = 0; i < MUMBER_LINE_LCD + 1; i ++)
		{
			tft_serial(i, lcd_text_color[*line - MUMBER_LINE_LCD  + i], lcd_data[*line - MUMBER_LINE_LCD  + i ]);
		}
	}
}*/
