#include "sofware_i2c.h"
//#include "main.h"

void delay_us (uint16_t us);

void I2C_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
	
	HAL_GPIO_WritePin(I2C_SDA_PORT, SDA_PIN, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = SDA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP ;       
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;;
  HAL_GPIO_Init(I2C_SDA_PORT, &GPIO_InitStruct);
	
	HAL_GPIO_WritePin(I2C_SCL_PORT, SCL_PIN, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = SCL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP ;       
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(I2C_SCL_PORT, &GPIO_InitStruct);
	
	HAL_GPIO_WritePin(GPIOC, SDA_PIN, GPIO_PIN_SET);
	delay_us(5);
	//HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOC, SCL_PIN, GPIO_PIN_SET);
	delay_us(5);
	//HAL_Delay(1);
}

void I2C_Start(void)
{
	HAL_GPIO_WritePin(I2C_SDA_PORT, SDA_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(I2C_SCL_PORT, SCL_PIN, GPIO_PIN_SET);
	delay_us(5);
	//HAL_Delay(1);
	HAL_GPIO_WritePin(I2C_SDA_PORT, SDA_PIN, GPIO_PIN_RESET);
	delay_us(5);
	//HAL_Delay(1);
	HAL_GPIO_WritePin(I2C_SCL_PORT, SCL_PIN, GPIO_PIN_RESET);
	//delay_us(5);
	//HAL_Delay(1);
}

void I2C_Stop(void)
{
	HAL_GPIO_WritePin(I2C_SDA_PORT, SDA_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(I2C_SCL_PORT, SCL_PIN, GPIO_PIN_SET);
	delay_us(5);
	//HAL_Delay(1);
	HAL_GPIO_WritePin(I2C_SDA_PORT, SDA_PIN, GPIO_PIN_SET);
	delay_us(5);
	//HAL_Delay(1);
	HAL_GPIO_WritePin(I2C_SCL_PORT, SCL_PIN, GPIO_PIN_RESET);
	//delay_us(5);
	//HAL_Delay(1);
}

unsigned char	I2C_checkack(void)
{
	unsigned char ack = 0;
	HAL_GPIO_WritePin(I2C_SDA_PORT, SDA_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(I2C_SCL_PORT, SCL_PIN, GPIO_PIN_SET);
	delay_us(5);
	ack = HAL_GPIO_ReadPin(I2C_SDA_PORT, SDA_PIN);
	delay_us(5);
	HAL_GPIO_WritePin(I2C_SCL_PORT, SCL_PIN, GPIO_PIN_RESET);
	delay_us(5);
	if(ack == 1) return 0;
	return 1;
}

void I2C_Write(unsigned char data)
{
	for(int i = 0; i < 8 ; i++)
	{
		HAL_GPIO_WritePin(I2C_SDA_PORT, SDA_PIN , (data<<i)&0x80);
		HAL_GPIO_WritePin(I2C_SCL_PORT, SCL_PIN, GPIO_PIN_SET);
		delay_us(5);
		HAL_GPIO_WritePin(I2C_SCL_PORT, SCL_PIN, GPIO_PIN_RESET);
		delay_us(5);
	}
}

unsigned char I2C_Read(void)
{
	unsigned char I2C_data= 0, i, temp;
	for(i = 0; i <8; i++)
	{
		HAL_GPIO_WritePin(I2C_SCL_PORT, SCL_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(I2C_SDA_PORT, SDA_PIN, GPIO_PIN_SET);
		delay_us(5);
		temp = HAL_GPIO_ReadPin(I2C_SDA_PORT, SDA_PIN);
		delay_us(5);
		HAL_GPIO_WritePin(I2C_SCL_PORT, SCL_PIN, GPIO_PIN_RESET);
		if(temp == 1)
		{
			I2C_data = I2C_data	 << 1;
			I2C_data = I2C_data|0x01;
		}
		else I2C_data=I2C_data<<1;
	}
	return I2C_data;
}

void I2C_WriteByte_24LC(uint16_t address, uint8_t data)
{
	uint8_t AckTemp = 1;
	I2C_Start();
	I2C_Write(EEPROM_ADDRESS);
	AckTemp = I2C_checkack();
	I2C_Write((uint8_t)address >> 8 & 0xff);
	AckTemp = I2C_checkack();
	I2C_Write((uint8_t)address & 0xff);
	AckTemp = I2C_checkack();
	I2C_Write(data);
	AckTemp = I2C_checkack();

	HAL_GPIO_WritePin(I2C_SDA_PORT, SDA_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(I2C_SCL_PORT, SCL_PIN, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(I2C_SDA_PORT, SDA_PIN, GPIO_PIN_SET);	
	HAL_Delay(1);
	HAL_GPIO_WritePin(I2C_SCL_PORT, SCL_PIN, GPIO_PIN_RESET);
	HAL_Delay(1);
}

uint8_t I2C_ReadByte_24LC(uint16_t address)
{
	uint8_t AckTemp = 1;
	uint8_t data;
	I2C_Start();
	I2C_Write(EEPROM_ADDRESS);
	AckTemp=I2C_checkack();
	I2C_Write((uint8_t)address>>8 & 0xff);
	AckTemp = I2C_checkack();
	I2C_Write((uint8_t)address & 0xff);
	AckTemp = I2C_checkack();
	I2C_Start();
  I2C_Write(EEPROM_ADDRESS|0x01);
	AckTemp=I2C_checkack();
	GPIO_I2CReadByte(&data);
  AckTemp=I2C_checkack();

	HAL_GPIO_WritePin(I2C_SDA_PORT, SDA_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(I2C_SCL_PORT, SCL_PIN, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(I2C_SDA_PORT, SDA_PIN, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(I2C_SCL_PORT, SCL_PIN, GPIO_PIN_RESET);
	HAL_Delay(1);

	return data;
}
void GPIO_I2CReadByte(uint8_t *Data)
{
	 int i, ReceivedData;

    *Data = 0;

    for(i=7; i>=0; i--)
    {
        HAL_GPIO_WritePin(I2C_SCL_PORT, SCL_PIN, GPIO_PIN_RESET);
        delay_us(5);
        HAL_GPIO_WritePin(I2C_SCL_PORT ,SCL_PIN, GPIO_PIN_SET);
        delay_us(5);
        ReceivedData = HAL_GPIO_ReadPin(I2C_SDA_PORT, SDA_PIN);
        if(ReceivedData > 0)
				*Data |= (0x1 << i);
        delay_us(5);
    }
}

uint8_t I2C_Check_Device_Connect(uint8_t addr)
{
	uint8_t check;
	I2C_Start();
	I2C_Write(addr);
	check = I2C_checkack();
	return check;
}

uint8_t I2C_Read_MCP(uint8_t ack)
{
	uint8_t bit;
    uint8_t byte = 0;

    HAL_GPIO_WritePin(I2C_SDA_PORT, SDA_PIN, GPIO_PIN_SET);

    for (uint8_t i = 0; i < 8; i++) {
        HAL_GPIO_WritePin(I2C_SCL_PORT, SCL_PIN, GPIO_PIN_SET);
        delay_us(5);
        bit = HAL_GPIO_ReadPin(I2C_SDA_PORT, SDA_PIN);
        byte = (byte << 1) | bit;
        HAL_GPIO_WritePin(I2C_SCL_PORT, SCL_PIN, GPIO_PIN_RESET);
        delay_us(5);
    }

    if (ack == 1) 
		{
        I2C_Ack();
    }
    else 
		{
        I2C_Nack();
    }

    return byte;
}

void I2C_Nack(void) {
    HAL_GPIO_WritePin(I2C_SDA_PORT, SDA_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(I2C_SCL_PORT, SCL_PIN, GPIO_PIN_SET);
    delay_us(5);
    HAL_GPIO_WritePin(I2C_SCL_PORT, SCL_PIN, GPIO_PIN_RESET);
}

void I2C_Ack(void) {
    HAL_GPIO_WritePin(I2C_SDA_PORT, SDA_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(I2C_SCL_PORT, SCL_PIN, GPIO_PIN_SET);
    delay_us(5);
    HAL_GPIO_WritePin(I2C_SCL_PORT, SCL_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(I2C_SDA_PORT, SDA_PIN, GPIO_PIN_SET);
}

