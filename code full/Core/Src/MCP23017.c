#include "MCP23017.h"




void SetInput_Gpio_A(uint8_t op_device)
{
    uint8_t ack = 1;
    I2C_Start();
    I2C_Write(op_device);
    ack = I2C_checkack();
    I2C_Write(REGISTER_IODIRA);
    ack = I2C_checkack();
    I2C_Write(0xff);
    ack = I2C_checkack();
    I2C_Stop();
}

void SetInput_Gpio_B(uint8_t op_device)
{
    uint8_t ack = 1;
    I2C_Start();
    I2C_Write(op_device);
    ack = I2C_checkack();
    I2C_Write(REGISTER_IODIRB);
    ack = I2C_checkack();
    I2C_Write(0xff);
    ack = I2C_checkack();
    I2C_Stop();
}

void SetOutput_Gpio_A(uint8_t op_device)
{
    uint8_t ack = 1;
    I2C_Start();
    I2C_Write(op_device);
    ack = I2C_checkack();
    I2C_Write(REGISTER_IODIRA);
    ack = I2C_checkack();
    I2C_Write(0x00);
    ack = I2C_checkack();
    I2C_Stop();
		I2C_WriteByte_Register(op_device, REGISTER_GPIOA, 0xff);
}

void SetOutput_Gpio_B(uint8_t op_device)
{
    uint8_t ack = 1;
    I2C_Start();
    I2C_Write(op_device);
    ack = I2C_checkack();
    I2C_Write(REGISTER_IODIRB);
    ack = I2C_checkack();
    I2C_Write(0x00);
    ack = I2C_checkack();
    I2C_Stop();
		I2C_WriteByte_Register(op_device, REGISTER_GPIOB, 0xff);
}

void DigitalWrite_Gpio_A(uint8_t op_device, uint8_t dataout)
{
    uint8_t ack = 1;
    I2C_Start();
    I2C_Write(op_device);
    ack = I2C_checkack();
    I2C_Write(REGISTER_GPIOA);
    ack = I2C_checkack();
    I2C_Write(dataout);
    ack = I2C_checkack();
    I2C_Stop();
}

void DigitalWrite_Gpio_B(uint8_t op_device,uint8_t dataout)
{
    uint8_t ack = 1;
    I2C_Start();
    I2C_Write(op_device);
    ack = I2C_checkack();
    I2C_Write(REGISTER_GPIOB);
    ack = I2C_checkack();
    I2C_Write(dataout);
    ack = I2C_checkack();
    I2C_Stop();
}

void I2C_WriteByte_Register(uint8_t op_device, uint8_t addr, uint8_t value)
{
    uint8_t ack = 1;
    I2C_Start();
    I2C_Write(op_device);
    ack = I2C_checkack();
    I2C_Write(addr);
    ack = I2C_checkack();
    I2C_Write(value);
	  ack = I2C_checkack();
    I2C_Stop();
}

uint8_t I2C_Read_Register(uint8_t op_device, uint8_t addr)
{
	uint16_t data1;
	uint8_t ack = 1;
  uint8_t data_out;
	I2C_Start();
	I2C_Write(op_device);
	ack = I2C_checkack();
	I2C_Write(addr);
	ack = I2C_checkack();
	I2C_Stop();
	
	I2C_Start(); 
	I2C_Write(op_device|0x01);
	ack = I2C_checkack();
	data1 = I2C_Read_MCP(1) << 8;
  data1 |= I2C_Read_MCP(1);
	I2C_Stop();
	
  data_out = (data1>>8)&0xFF;
	return data_out;
}

void DigitalWrite_Pin(uint8_t op_device, uint8_t GPIO, uint8_t PIN, uint8_t Dout)
{
    uint8_t data_out;
    uint8_t data_register;
    data_register = I2C_Read_Register(op_device,GPIO);
    if(Dout == HIGH) data_out = ((Dout<<PIN)|0x00)|data_register;
    else data_out = ~((!Dout<<PIN)|0x00) & data_register;
    I2C_WriteByte_Register(op_device, GPIO, data_out);
}


























