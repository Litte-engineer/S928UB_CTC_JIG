#include "Spen_74hc.h"



void WriteByte_74HC (uint8_t u ,uint8_t data);
void WriteBit_74HC (uint8_t u, uint8_t PIN, uint8_t bit);
void (*test_con)(uint8_t, uint8_t, uint8_t) = &WriteBit_74HC;
	

void WriteByte_74HC (uint8_t u ,uint8_t data);
void WriteBit_74HC (uint8_t u, uint8_t PIN, uint8_t bit);

void WriteByte_74HC (uint8_t U ,uint8_t data)
{
    //SetOutput_Gpio_A(MCP23017_DEVICE_1);
    I2C_WriteByte_Register(MCP23017_DEVICE_1, REGISTER_GPIOA, data);
    switch (U)
    {
        case 1 :
            //SetOutput_Gpio_A(MCP23017_DEVICE_2);
            DigitalWrite_Pin(MCP23017_DEVICE_1, REGISTER_GPIOB, PIN_0, LOW);
            DigitalWrite_Pin(MCP23017_DEVICE_1, REGISTER_GPIOB, PIN_0, HIGH);
        break;

        case 2 :
            //SetOutput_Gpio_A(MCP23017_DEVICE_2);
            DigitalWrite_Pin(MCP23017_DEVICE_1, REGISTER_GPIOB, PIN_1, LOW);
            DigitalWrite_Pin(MCP23017_DEVICE_1, REGISTER_GPIOB, PIN_1, HIGH);
        break;

        case 3 :
            //SetOutput_Gpio_B(MCP23017_DEVICE_1);
            DigitalWrite_Pin(MCP23017_DEVICE_1, REGISTER_GPIOB, PIN_2, LOW);
            DigitalWrite_Pin(MCP23017_DEVICE_1, REGISTER_GPIOB, PIN_2, HIGH);
        break;

        case 4 :
            //SetOutput_Gpio_B(MCP23017_DEVICE_1);
            DigitalWrite_Pin(MCP23017_DEVICE_1, REGISTER_GPIOB, PIN_3, LOW);
            DigitalWrite_Pin(MCP23017_DEVICE_1, REGISTER_GPIOB, PIN_3, HIGH);
        break;

        case 5 :
            //SetOutput_Gpio_B(MCP23017_DEVICE_1);
            DigitalWrite_Pin(MCP23017_DEVICE_1, REGISTER_GPIOB, PIN_5, LOW);
            DigitalWrite_Pin(MCP23017_DEVICE_1, REGISTER_GPIOB, PIN_5, HIGH);
        break;

        case 6 :
            //SetOutput_Gpio_B(MCP23017_DEVICE_2);
            DigitalWrite_Pin(MCP23017_DEVICE_1, REGISTER_GPIOB, PIN_6, LOW);
            DigitalWrite_Pin(MCP23017_DEVICE_1, REGISTER_GPIOB, PIN_6, HIGH);
        break;

        case 7 :
            //SetOutput_Gpio_B(MCP23017_DEVICE_2);
            DigitalWrite_Pin(MCP23017_DEVICE_1, REGISTER_GPIOB, PIN_7, LOW);
            DigitalWrite_Pin(MCP23017_DEVICE_1, REGISTER_GPIOB, PIN_7, HIGH);
        break;

        case 8 :
            //SetOutput_Gpio_B(MCP23017_DEVICE_2);
            DigitalWrite_Pin(MCP23017_DEVICE_1, REGISTER_GPIOB, PIN_4, LOW);
            DigitalWrite_Pin(MCP23017_DEVICE_1, REGISTER_GPIOB, PIN_4, HIGH);
        break;

        case 9 :
            //SetOutput_Gpio_B(MCP23017_DEVICE_2);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOA, PIN_3, LOW);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOA, PIN_3, HIGH);
        break;

        case 10 :
            //SetOutput_Gpio_A(MCP23017_DEVICE_2);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOA, PIN_0, LOW);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOA, PIN_0, HIGH);
        break;

        case 11 :
            //SetOutput_Gpio_B(MCP23017_DEVICE_1);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOA, PIN_1, LOW);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOA, PIN_1, HIGH);
        break;

        case 12 :
            //SetOutput_Gpio_B(MCP23017_DEVICE_1);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOA, PIN_2, LOW);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOA, PIN_2, HIGH);
        break;

        case 13 :
            //SetOutput_Gpio_B(MCP23017_DEVICE_1);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOA, PIN_7, LOW);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOA, PIN_7, HIGH);
        break;

        case 14 :
            //SetOutput_Gpio_B(MCP23017_DEVICE_1);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOA, PIN_4, LOW);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOA, PIN_4, HIGH);
        break;

        case 15 :
            //SetOutput_Gpio_B(MCP23017_DEVICE_1);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOA, PIN_5, LOW);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOA, PIN_5, HIGH);
        break;

        case 16 :
            //SetOutput_Gpio_A(MCP23017_DEVICE_2);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOA, PIN_6, LOW);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOA, PIN_6, HIGH);
        break;

        case 17 :
            //SetOutput_Gpio_A(MCP23017_DEVICE_2);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOB, PIN_3, LOW);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOB, PIN_3, HIGH);
        break;

        case 18 :
            //SetOutput_Gpio_A(MCP23017_DEVICE_2);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOB, PIN_0, LOW);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOB, PIN_0, HIGH);
        break;

        case 19 :
            //SetOutput_Gpio_A(MCP23017_DEVICE_2);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOB, PIN_1, LOW);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOB, PIN_1, HIGH);
        break;

        case 20 :
            //SetOutput_Gpio_A(MCP23017_DEVICE_2);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOB, PIN_2, LOW);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOB, PIN_2, HIGH);
        break;
    }
}


void WriteBit_74HC (uint8_t U, uint8_t PIN, uint8_t bit)
{
    //SetOutput_Gpio_A(MCP23017_DEVICE_1);
    switch (PIN)
    {
        case 1 :
					DigitalWrite_Pin(MCP23017_DEVICE_1, REGISTER_GPIOA, PIN_0, bit);
				break;

				case 2 :
					DigitalWrite_Pin(MCP23017_DEVICE_1, REGISTER_GPIOA, PIN_1, bit);
				break;

				case 3 :
					DigitalWrite_Pin(MCP23017_DEVICE_1, REGISTER_GPIOA, PIN_2, bit);
				break;

				case 4 :
					DigitalWrite_Pin(MCP23017_DEVICE_1, REGISTER_GPIOA, PIN_3, bit);
				break;
			
				case 5 :
					DigitalWrite_Pin(MCP23017_DEVICE_1, REGISTER_GPIOA, PIN_4, bit);
				break;
				
				case 6 :
					DigitalWrite_Pin(MCP23017_DEVICE_1, REGISTER_GPIOA, PIN_5, bit);
				break;
			
				case 7 :
					DigitalWrite_Pin(MCP23017_DEVICE_1, REGISTER_GPIOA, PIN_6, bit);
				break;
			
				case 8 :
					DigitalWrite_Pin(MCP23017_DEVICE_1, REGISTER_GPIOA, PIN_7, bit);
				break;		
    }





    switch (U)
    {
       case 1 :
            //SetOutput_Gpio_A(MCP23017_DEVICE_2);
            DigitalWrite_Pin(MCP23017_DEVICE_1, REGISTER_GPIOB, PIN_0, LOW);
            DigitalWrite_Pin(MCP23017_DEVICE_1, REGISTER_GPIOB, PIN_0, HIGH);
        break;

        case 2 :
            //SetOutput_Gpio_A(MCP23017_DEVICE_2);
            DigitalWrite_Pin(MCP23017_DEVICE_1, REGISTER_GPIOB, PIN_1, LOW);
            DigitalWrite_Pin(MCP23017_DEVICE_1, REGISTER_GPIOB, PIN_1, HIGH);
        break;

        case 3 :
            //SetOutput_Gpio_B(MCP23017_DEVICE_1);
            DigitalWrite_Pin(MCP23017_DEVICE_1, REGISTER_GPIOB, PIN_2, LOW);
            DigitalWrite_Pin(MCP23017_DEVICE_1, REGISTER_GPIOB, PIN_2, HIGH);
        break;

        case 4 :
            //SetOutput_Gpio_B(MCP23017_DEVICE_1);
            DigitalWrite_Pin(MCP23017_DEVICE_1, REGISTER_GPIOB, PIN_3, LOW);
            DigitalWrite_Pin(MCP23017_DEVICE_1, REGISTER_GPIOB, PIN_3, HIGH);
        break;

        case 5 :
            //SetOutput_Gpio_B(MCP23017_DEVICE_1);
            DigitalWrite_Pin(MCP23017_DEVICE_1, REGISTER_GPIOB, PIN_5, LOW);
            DigitalWrite_Pin(MCP23017_DEVICE_1, REGISTER_GPIOB, PIN_5, HIGH);
        break;

        case 6 :
            //SetOutput_Gpio_B(MCP23017_DEVICE_2);
            DigitalWrite_Pin(MCP23017_DEVICE_1, REGISTER_GPIOB, PIN_6, LOW);
            DigitalWrite_Pin(MCP23017_DEVICE_1, REGISTER_GPIOB, PIN_6, HIGH);
        break;

        case 7 :
            //SetOutput_Gpio_B(MCP23017_DEVICE_2);
            DigitalWrite_Pin(MCP23017_DEVICE_1, REGISTER_GPIOB, PIN_7, LOW);
            DigitalWrite_Pin(MCP23017_DEVICE_1, REGISTER_GPIOB, PIN_7, HIGH);
        break;

        case 8 :
            //SetOutput_Gpio_B(MCP23017_DEVICE_2);
            DigitalWrite_Pin(MCP23017_DEVICE_1, REGISTER_GPIOB, PIN_4, LOW);
            DigitalWrite_Pin(MCP23017_DEVICE_1, REGISTER_GPIOB, PIN_4, HIGH);
        break;

        case 9 :
            //SetOutput_Gpio_B(MCP23017_DEVICE_2);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOA, PIN_3, LOW);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOA, PIN_3, HIGH);
        break;

        case 10 :
            //SetOutput_Gpio_A(MCP23017_DEVICE_2);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOA, PIN_0, LOW);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOA, PIN_0, HIGH);
        break;

        case 11 :
            //SetOutput_Gpio_B(MCP23017_DEVICE_1);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOA, PIN_1, LOW);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOA, PIN_1, HIGH);
        break;

        case 12 :
            //SetOutput_Gpio_B(MCP23017_DEVICE_1);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOA, PIN_2, LOW);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOA, PIN_2, HIGH);
        break;

        case 13 :
            //SetOutput_Gpio_B(MCP23017_DEVICE_1);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOA, PIN_7, LOW);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOA, PIN_7, HIGH);
        break;

        case 14 :
            //SetOutput_Gpio_B(MCP23017_DEVICE_1);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOA, PIN_4, LOW);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOA, PIN_4, HIGH);
        break;

        case 15 :
            //SetOutput_Gpio_B(MCP23017_DEVICE_1);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOA, PIN_5, LOW);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOA, PIN_5, HIGH);
        break;

        case 16 :
            //SetOutput_Gpio_A(MCP23017_DEVICE_2);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOA, PIN_6, LOW);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOA, PIN_6, HIGH);
        break;

        case 17 :
            //SetOutput_Gpio_A(MCP23017_DEVICE_2);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOB, PIN_3, LOW);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOB, PIN_3, HIGH);
        break;

        case 18 :
            //SetOutput_Gpio_A(MCP23017_DEVICE_2);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOB, PIN_0, LOW);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOB, PIN_0, HIGH);
        break;

        case 19 :
            //SetOutput_Gpio_A(MCP23017_DEVICE_2);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOB, PIN_1, LOW);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOB, PIN_1, HIGH);
        break;

        case 20 :
            //SetOutput_Gpio_A(MCP23017_DEVICE_2);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOB, PIN_2, LOW);
            DigitalWrite_Pin(MCP23017_DEVICE_2, REGISTER_GPIOB, PIN_2, HIGH);
        break;

    }
}



























