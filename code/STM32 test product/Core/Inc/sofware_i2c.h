#ifndef __SOFWARE_I2C_H
#define __SOFWARE_I2C_H

#include "main.h"


#define EEPROM_ADDRESS 0xA8
#define SDA_PIN GPIO_PIN_5 
#define SCL_PIN GPIO_PIN_6
#define I2C_SDA_PORT GPIOC
#define I2C_SCL_PORT GPIOC

void I2C_Init(void);
void I2C_Start(void);
void I2C_Stop(void);
unsigned char	I2C_checkack(void);
void I2C_Write(unsigned char data);
unsigned char I2C_Read(void);
void I2C_WriteByte_24LC(uint16_t address, uint8_t data);
uint8_t I2C_ReadByte_24LC(uint16_t address);
void GPIO_I2CReadByte(uint8_t *Data);
uint8_t I2C_Check_Device_Connect(uint8_t addr);
void I2C_Ack(void);
void I2C_Nack(void);
uint8_t I2C_Read_MCP(uint8_t ack);

#endif 
