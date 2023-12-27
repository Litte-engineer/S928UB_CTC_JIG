#ifndef __MCP23017_H
#define __MCP23017_H


#include "sofware_i2c.h"

#define MCP23017_DEVICE_1 0x40
#define MCP23017_DEVICE_2 0x42
#define MCP23017_DEVICE_3 0x44

#define HIGH 1
#define LOW 0

#define PIN_0 0
#define PIN_1 1
#define PIN_2 2
#define PIN_3 3
#define PIN_4 4
#define PIN_5 5
#define PIN_6 6
#define PIN_7 7

#define REGISTER_IODIRA		0x00
#define REGISTER_IODIRB		0x01
#define REGISTER_IPOLA		0x02
#define REGISTER_IPOLB		0x03
#define REGISTER_GPINTENA	0x04
#define REGISTER_GPINTENB	0x05
#define REGISTER_DEFVALA	0x06
#define REGISTER_DEFVALB	0x07
#define REGISTER_INTCONA	0x08
#define REGISTER_INTCONB	0x09

#define REGISTER_GPPUA		0x0C
#define REGISTER_GPPUB		0x0D
#define REGISTER_INTFA		0x0E
#define REGISTER_INTFB		0x0F
#define REGISTER_INTCAPA	0x10
#define REGISTER_INTCAPB	0x11
#define REGISTER_GPIOA		0x12
#define REGISTER_GPIOB		0x13
#define REGISTER_OLATA		0x14
#define REGISTER_OLATB		0x15

void SetInput_Gpio_A(uint8_t op_device);
void SetInput_Gpio_B(uint8_t op_device);
void SetOutput_Gpio_A(uint8_t op_device);
void SetOutput_Gpio_B(uint8_t op_device);
uint8_t I2C_Read_Register(uint8_t op_device, uint8_t addr);
void I2C_WriteByte_Register(uint8_t op_device, uint8_t addr, uint8_t value);
void DigitalWrite_Pin(uint8_t op_device, uint8_t GPIO, uint8_t PIN, uint8_t Dout);

#endif








