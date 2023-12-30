/*
 * interrupt_callback.c
 *
 *  Created on: 2019. 3. 6.
 *      Author: SEO DONG JIN
 */

#include "main.h"

extern uint8_t gUart1_Rx_Buff[10];
extern uint8_t gUart2_Rx_Buff[1];
extern uint8_t key_input;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern volatile pba_bool tft_state;
extern volatile pba_bool touch_state;
extern volatile pba_bool Rst_Flag;
extern uint8_t g_touch_buff[9];
extern uint8_t lcd_buffer[300];

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  if(huart->Instance == USART1)
  {
	  if(tft_state == pba_false)
	  {
		tft_state = pba_true;

		for(int i=0; i<10; i++)
		{
			sprintf((char*)lcd_buffer, "[0x%2X]", (char)gUart1_Rx_Buff[i]);
			Debug_serial((char*)lcd_buffer);
		}
		Debug_serial("\r\n");

		HAL_UART_Receive_DMA(&huart1, gUart1_Rx_Buff, 9); //9byte
	  }
	  else
	  {
#if 1
		  for(int i=0; i<9; i++)
		  {
			  sprintf((char*)lcd_buffer, "[0x%2X]", (char)gUart1_Rx_Buff[i]);
			  Debug_serial((char*)lcd_buffer);
		  }
		  Debug_serial("\r\n");
#endif
		  touch_state = pba_true;
		  memcpy(g_touch_buff, gUart1_Rx_Buff, 9);
	  }
  }

  else if(huart->Instance == USART2)
  {
	  HAL_UART_Receive_DMA(&huart2, gUart2_Rx_Buff, 1);
  }
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_UART_TxCpltCallback could be implemented in the user file
   */
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == RESET_KEY_Pin)//S/W Reset
	{
		if(HAL_GPIO_ReadPin(RESET_KEY_GPIO_Port, RESET_KEY_Pin) == GPIO_PIN_RESET)
		{
			//while(HAL_GPIO_ReadPin(RESET_KEY_GPIO_Port, RESET_KEY_Pin) == GPIO_PIN_RESET);
			Rst_Flag = pba_true;
		}
	}
}
