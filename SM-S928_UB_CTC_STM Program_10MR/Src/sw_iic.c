/*
 * sw_iic.c
 *
 *  Created on: 2019. 3. 11.
 *      Author: synopex
 */

#include "main.h"

#define SDA_HIGH()		SDA_InMode()
#define SDA_LOW()       SDA_OutMode()
#define SCL_HIGH()      SCL_InMode()
#define SCL_LOW()       SCL_OutMode()
#define RETRY_CNT 200

GPIO_InitTypeDef GPIO_Init_value;

static void iic_delay_us(void);
static uint8_t SDA_Value(void);
static void GPIO_I2CStart(void);
static void GPIO_I2CStop(void);
static uint8_t GPIO_I2CIsAck(void);
static void GPIO_I2COutNoAck(void);
static void GPIO_I2COutAck(void);
static void GPIO_I2CWriteByte(uint8_t Data);
static void GPIO_I2CReadByte(uint8_t *Data);
static int16_t mcp23017_i2c_write(uint8_t SLAVE_ADDR, uint16_t Addr, uint8_t pData);
static int16_t mcp23017_i2c_read(uint8_t SLAVE_ADDR, uint16_t Addr, uint8_t* pData, uint32_t Cnt);
static void Set_Bit(uint8_t slv_addr, uint8_t port, uint8_t value);
static void Clear_Bit(uint8_t slv_addr, uint8_t port, uint8_t value);

static void iic_delay_us(void)
{
	uint8_t d = 49;

	while(d--) //5-us
	{
		asm volatile("nop");
	}
}

void SDA_OutMode(void)
{
	GPIO_Init_value.Pin = SW_SDA_Pin;
	GPIO_Init_value.Mode = GPIO_MODE_OUTPUT_OD; //OUTPUT OPEN DRAIN
	GPIO_Init_value.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SW_SDA_GPIO_Port, &GPIO_Init_value);
}

void SCL_OutMode(void)
{
	GPIO_Init_value.Pin = SW_SCL_Pin;
	GPIO_Init_value.Mode = GPIO_MODE_OUTPUT_OD; //OUTPUT OPEN DRAIN
	GPIO_Init_value.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SW_SCL_GPIO_Port, &GPIO_Init_value);
}

void SDA_InMode(void)
{
	GPIO_Init_value.Pin = SW_SDA_Pin;
	GPIO_Init_value.Mode = GPIO_MODE_INPUT;
	GPIO_Init_value.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(SW_SDA_GPIO_Port, &GPIO_Init_value);
}

void SCL_InMode(void)
{
	GPIO_Init_value.Pin = SW_SCL_Pin;
	GPIO_Init_value.Mode = GPIO_MODE_INPUT;
	GPIO_Init_value.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(SW_SCL_GPIO_Port, &GPIO_Init_value);
}

static uint8_t SDA_Value(void)
{
    return HAL_GPIO_ReadPin(SW_SDA_GPIO_Port, SW_SDA_Pin);
}

static void GPIO_I2CStart(void)
{
    SDA_HIGH();
    SCL_HIGH();
    iic_delay_us();

    SDA_LOW();

    iic_delay_us();

    SCL_LOW();
}

static void GPIO_I2CStop(void)
{
    SDA_HIGH();
    iic_delay_us();

    SCL_LOW();
    iic_delay_us();

    SDA_LOW();
    iic_delay_us();

    SCL_HIGH();
    iic_delay_us();

    SDA_HIGH();
    iic_delay_us();

}

static uint8_t GPIO_I2CIsAck(void)
{
    int ReceivedData;

    SCL_LOW();
    SDA_InMode();
    iic_delay_us();

    SCL_HIGH();
    iic_delay_us();

    ReceivedData = SDA_Value();

    SCL_LOW();
    SDA_LOW();
    iic_delay_us();

    if(ReceivedData == 0)
		return 1;
    else
		return 0;
}

static void GPIO_I2COutNoAck(void)
{
    SCL_LOW();
    SCL_LOW();	//dummy delay

    SDA_HIGH();
    iic_delay_us();

    SCL_HIGH();
    iic_delay_us();

    SCL_LOW();
    iic_delay_us();
}

static void GPIO_I2COutAck(void)
{
    SCL_LOW();
    SCL_LOW();	//Dummy delay

    SDA_LOW();
    iic_delay_us();

    SCL_HIGH();
    iic_delay_us();

    SCL_LOW();
    iic_delay_us();
}

static void GPIO_I2CWriteByte(uint8_t Data)
{
    int i = 0;

    for(i=7; i>=0; i--)
    {
        SCL_LOW();

        if(Data & (0x01<<i))
            SDA_HIGH();
        else
            SDA_LOW();

        iic_delay_us();

        SCL_HIGH();

        iic_delay_us();
    }
}

static void GPIO_I2CReadByte(uint8_t *Data)
{
    int i, ReceivedData;

    *Data = 0;

    for(i=7; i>=0; i--)
    {
        SCL_LOW();
        SDA_InMode();
        iic_delay_us();

        SCL_HIGH();
        iic_delay_us();

        ReceivedData = SDA_Value();

        if(ReceivedData > 0)
			*Data |= (0x1 << i);

        iic_delay_us();
    }
}

static int16_t mcp23017_i2c_write(uint8_t SLAVE_ADDR, uint16_t Addr, uint8_t pData)
{
	int16_t rv = 0;
	uint16_t Retry = 0;
	uint8_t TempData;

RETRY:

	Retry++;
	if(Retry > RETRY_CNT)
	{
		LCD_DbgLog("WriteRegister [Err] Write Ack fail!\n");
		return -1;
	}

	GPIO_I2CStart();

	GPIO_I2CWriteByte(SLAVE_ADDR & 0xFE);
	if(GPIO_I2CIsAck() == 0)
	{
		GPIO_I2CStop();
		goto RETRY;
	}

	//Device address write...
	TempData = (uint8_t)(Addr & 0x00FF);

	GPIO_I2CWriteByte(TempData);
	if(GPIO_I2CIsAck() == 0)
	{
		GPIO_I2CStop();
		goto RETRY;
	}

	//Send 1-byte Data
	GPIO_I2CWriteByte(pData);
	if(GPIO_I2CIsAck() == 0)
	{
		goto error;
	}
	else
		rv++;

	GPIO_I2CStop();

	return rv;

error:
	return -1;
}

static int16_t mcp23017_i2c_read(uint8_t SLAVE_ADDR, uint16_t Addr, uint8_t* pData, uint32_t Cnt)
{
	int16_t rv = 0;
	uint16_t Retry = 0;
	uint8_t TempData8;

	RETRY:

	Retry++;
	if(Retry > RETRY_CNT)
	{
		LCD_DbgLog("[Err] Read Ack Fail!\n");
		return -1;
	}

	GPIO_I2CStart();

	GPIO_I2CWriteByte(SLAVE_ADDR & 0xFE);
	if(GPIO_I2CIsAck() == 0)
	{
		GPIO_I2CStop();
		goto RETRY;
	}

	//Device Address 8bit
	TempData8 = (uint8_t)(Addr & 0x00FF);

	GPIO_I2CWriteByte(TempData8);
	if(GPIO_I2CIsAck() == 0)
	{
		GPIO_I2CStop();
		goto RETRY;
	}
/*
	GPIO_I2CStop();
	_delay_us(5);
*/
	GPIO_I2CStart(); //I2C Restart

	GPIO_I2CWriteByte((SLAVE_ADDR & 0xFE) | 0x01);
	if(GPIO_I2CIsAck() == 0)
	{
		GPIO_I2CStop();
		goto RETRY;
	}

	for(uint8_t i=0; i<Cnt; i++)
	{
		GPIO_I2CReadByte(&TempData8);

		*pData++ = TempData8;
		rv++;
		if(i==(Cnt-1))
		{
			GPIO_I2COutNoAck();
			i = Cnt;	// force end of loop
			break;
		}
		else
		{
			GPIO_I2COutAck();
		}
	}

	GPIO_I2CStop();

	return rv;
}

int mcp23017_init(uint8_t slave_num)
{
	uint8_t rbuf[22];
	uint8_t device_address = slave_num;

	if(mcp23017_i2c_write(device_address, 0x0A, 0x80) != 1) //ICONA.BANK = 1
		goto err;
	if(mcp23017_i2c_write(device_address, 0x0B, 0x80) != 1) //ICONB.BANK = 1
		goto err;

	if(mcp23017_i2c_write(device_address, 0x00, 0x00) != 1) //PORTA ALL OUTPUT
		goto err;
	if(mcp23017_i2c_write(device_address, 0x10, 0x00) != 1) //PORTB ALL OUTPUT
		goto err;

	if(mcp23017_i2c_write(device_address, PA, 0x00) != 1) //PORTA ALL LOW
		goto err;
	if(mcp23017_i2c_write(device_address, PB, 0x00) != 1)  //PORTB ALL LOW
		goto err;

	if(mcp23017_i2c_read(device_address, 0x05, &rbuf[0], 1) != 1) //ICONA
		goto err;
	if(mcp23017_i2c_read(device_address, 0x15, &rbuf[1], 1) != 1) //ICONB
		goto err;

	if((rbuf[0] == 0x80) && (rbuf[1] == 0x80))
	{
		LCD_OkLog("0x%x CHIP Read OK\n", device_address);
		return 1;
	}
	else
	{
		LCD_ErrLog("0x%x CHIP Read Err\n", device_address);
		goto err;
	}

err:
	return -1;
}

int eep_256_burst_write(uint32_t addr, uint8_t *MemTarget, uint16_t Size)
{
	int16_t rv = 0;
	uint16_t Retry = 0;
	uint8_t slv_addr = 0xA8; //1010 1000
	union{
			uint32_t u16tmp;

			struct{
				uint8_t byte1; //lsb
				uint8_t byte2; //msb
			}byte;
		}tmp16;

	tmp16.u16tmp = (addr & 0x0000FFFF);

RETRY:

	Retry++;

	if(Retry > RETRY_CNT)
	{
		LCD_DbgLog("WriteRegister [Err] Write Ack fail!\n");
		return -1;
	}

	GPIO_I2CStart();

	GPIO_I2CWriteByte(slv_addr & 0xFE);
	if(GPIO_I2CIsAck() == 0)
	{
		GPIO_I2CStop();
		goto RETRY;
	}

	//Send Address High Byte
	GPIO_I2CWriteByte(tmp16.byte.byte2);

	if(GPIO_I2CIsAck() == 0)
	{
		goto error;
	}
	else
		rv++;

	//Send Address Lower Byte
	GPIO_I2CWriteByte(tmp16.byte.byte1);
	if(GPIO_I2CIsAck() == 0)
	{
		goto error;
	}
	else
		rv++;

	for(int a=0; a<Size; a++)
	{
		//Send Burst Data
		GPIO_I2CWriteByte(*MemTarget);
		if(GPIO_I2CIsAck() == 0)
		{
			goto error;
		}
		else
			rv++;

		*MemTarget++;
	}

	GPIO_I2CStop();

	return rv;

error:
	return -1;
}

int eep_256_burst_read(uint32_t addr, uint8_t *MemTarget, uint16_t Size)
{
	int16_t rv = 0;
	uint16_t Retry = 0;
	uint8_t slv_addr = 0xA8; //1010 0000
	uint8_t TempData8;
	union{
		uint32_t u16tmp;

		struct{
			uint8_t byte1; //lsb
			uint8_t byte2; //msb
		}byte;
	}tmp16;

	tmp16.u16tmp = (addr & 0x0000FFFF);

RETRY:

	Retry++;

	if(Retry > RETRY_CNT)
	{
		LCD_DbgLog("WriteRegister [Err] Write Ack fail!\n");
		return -1;
	}

	GPIO_I2CStart();

	GPIO_I2CWriteByte(slv_addr & 0xFE);
	if(GPIO_I2CIsAck() == 0)
	{
		GPIO_I2CStop();
		goto RETRY;
	}

	//Send Address High Byte
	GPIO_I2CWriteByte(tmp16.byte.byte2);
	if(GPIO_I2CIsAck() == 0)
	{
		goto error;
	}
	else
		rv++;

	//Send Address Lower Byte
	GPIO_I2CWriteByte(tmp16.byte.byte1);
	if(GPIO_I2CIsAck() == 0)
	{
		goto error;
	}
	else
		rv++;

	GPIO_I2CStart(); //Repeat Start

	GPIO_I2CWriteByte((slv_addr & 0xFE) | 0x01);
	if(GPIO_I2CIsAck() == 0)
	{
		GPIO_I2CStop();
		goto RETRY;
	}

	for(uint8_t i=0; i<Size; i++)
	{
		GPIO_I2CReadByte(&TempData8);

		*MemTarget++ = TempData8;
		rv++;
		if(i==(Size-1))
		{
			GPIO_I2COutNoAck();
			i = Size;	// force end of loop
			break;
		}
		else
		{
			GPIO_I2COutAck();
		}
	}

	GPIO_I2CStop();

	return rv;

error:
	return -1;
}

static void Set_Bit(uint8_t slv_addr, uint8_t port, uint8_t value)
{
	uint8_t rbuf = 0;
	uint8_t wbuf = 0;

	mcp23017_i2c_read(slv_addr, port, &rbuf, 1);
	if(value > 7) value = 0;

	wbuf = rbuf | (0x01 << value);

	mcp23017_i2c_write(slv_addr, port, wbuf);
}

static void Clear_Bit(uint8_t slv_addr, uint8_t port, uint8_t value)
{
	uint8_t rbuf = 0;
	uint8_t wbuf = 0;

	mcp23017_i2c_read(slv_addr, port, &rbuf, 1);

	if(value > 7) value = 0;

	wbuf = rbuf & ~(0x01 << value);

	mcp23017_i2c_write(slv_addr, port, wbuf);
}

void DFF_Data(uint8_t data)
{
	mcp23017_i2c_write(SLAVE0, PA, data);
}

//DFF1 ~ DFF20 => CH CNT-I
void DFF1(void){Clear_Bit(SLAVE0, PB, 0); DFF_DELAY; Set_Bit(SLAVE0, PB, 0); DFF_DELAY;}
void DFF2(void){Clear_Bit(SLAVE0, PB, 1); DFF_DELAY; Set_Bit(SLAVE0, PB, 1); DFF_DELAY;}
void DFF3(void){Clear_Bit(SLAVE0, PB, 2); DFF_DELAY; Set_Bit(SLAVE0, PB, 2); DFF_DELAY;}
void DFF4(void){Clear_Bit(SLAVE0, PB, 3); DFF_DELAY; Set_Bit(SLAVE0, PB, 3); DFF_DELAY;}
void DFF5(void){Clear_Bit(SLAVE0, PB, 4); DFF_DELAY; Set_Bit(SLAVE0, PB, 4); DFF_DELAY;}
void DFF6(void){Clear_Bit(SLAVE0, PB, 5); DFF_DELAY; Set_Bit(SLAVE0, PB, 5); DFF_DELAY;}
void DFF7(void){Clear_Bit(SLAVE0, PB, 6); DFF_DELAY; Set_Bit(SLAVE0, PB, 6); DFF_DELAY;}
void DFF8(void){Clear_Bit(SLAVE0, PB, 7); DFF_DELAY; Set_Bit(SLAVE0, PB, 7); DFF_DELAY;}
void DFF9(void){Clear_Bit(SLAVE1, PA, 0); DFF_DELAY; Set_Bit(SLAVE1, PA, 0); DFF_DELAY;}
void DFF10(void){Clear_Bit(SLAVE1, PA, 1); DFF_DELAY; Set_Bit(SLAVE1, PA, 1); DFF_DELAY;}
void DFF11(void){Clear_Bit(SLAVE1, PA, 2); DFF_DELAY; Set_Bit(SLAVE1, PA, 2); DFF_DELAY;}
void DFF12(void){Clear_Bit(SLAVE1, PA, 3); DFF_DELAY; Set_Bit(SLAVE1, PA, 3); DFF_DELAY;}
void DFF13(void){Clear_Bit(SLAVE1, PA, 4); DFF_DELAY; Set_Bit(SLAVE1, PA, 4); DFF_DELAY;}
void DFF14(void){Clear_Bit(SLAVE1, PA, 5); DFF_DELAY; Set_Bit(SLAVE1, PA, 5); DFF_DELAY;}
void DFF15(void){Clear_Bit(SLAVE1, PA, 6); DFF_DELAY; Set_Bit(SLAVE1, PA, 6); DFF_DELAY;}
void DFF16(void){Clear_Bit(SLAVE1, PA, 7); DFF_DELAY; Set_Bit(SLAVE1, PA, 7); DFF_DELAY;}
void DFF17(void){Clear_Bit(SLAVE1, PB, 0); DFF_DELAY; Set_Bit(SLAVE1, PB, 0); DFF_DELAY;}
void DFF18(void){Clear_Bit(SLAVE1, PB, 1); DFF_DELAY; Set_Bit(SLAVE1, PB, 1); DFF_DELAY;}
void DFF19(void){Clear_Bit(SLAVE1, PB, 2); DFF_DELAY; Set_Bit(SLAVE1, PB, 2); DFF_DELAY;}
void DFF20(void){Clear_Bit(SLAVE1, PB, 3); DFF_DELAY; Set_Bit(SLAVE1, PB, 3); DFF_DELAY;}

//DFF21 ~ DFF40 => CH CNT-II
void DFF21(void){Clear_Bit(SLAVE1, PB, 4); DFF_DELAY; Set_Bit(SLAVE1, PB, 4); DFF_DELAY;}
void DFF22(void){Clear_Bit(SLAVE1, PB, 5); DFF_DELAY; Set_Bit(SLAVE1, PB, 5); DFF_DELAY;}
void DFF23(void){Clear_Bit(SLAVE1, PB, 6); DFF_DELAY; Set_Bit(SLAVE1, PB, 6); DFF_DELAY;}
void DFF24(void){Clear_Bit(SLAVE1, PB, 7); DFF_DELAY; Set_Bit(SLAVE1, PB, 7); DFF_DELAY;}
void DFF25(void){Clear_Bit(SLAVE2, PA, 0); DFF_DELAY; Set_Bit(SLAVE2, PA, 0); DFF_DELAY;}
void DFF26(void){Clear_Bit(SLAVE2, PA, 1); DFF_DELAY; Set_Bit(SLAVE2, PA, 1); DFF_DELAY;}
void DFF27(void){Clear_Bit(SLAVE2, PA, 2); DFF_DELAY; Set_Bit(SLAVE2, PA, 2); DFF_DELAY;}
void DFF28(void){Clear_Bit(SLAVE2, PA, 3); DFF_DELAY; Set_Bit(SLAVE2, PA, 3); DFF_DELAY;}
void DFF29(void){Clear_Bit(SLAVE2, PA, 4); DFF_DELAY; Set_Bit(SLAVE2, PA, 4); DFF_DELAY;}
void DFF30(void){Clear_Bit(SLAVE2, PA, 5); DFF_DELAY; Set_Bit(SLAVE2, PA, 5); DFF_DELAY;}
void DFF31(void){Clear_Bit(SLAVE2, PA, 6); DFF_DELAY; Set_Bit(SLAVE2, PA, 6); DFF_DELAY;}
void DFF32(void){Clear_Bit(SLAVE2, PA, 7); DFF_DELAY; Set_Bit(SLAVE2, PA, 7); DFF_DELAY;}
void DFF33(void){Clear_Bit(SLAVE2, PB, 0); DFF_DELAY; Set_Bit(SLAVE2, PB, 0); DFF_DELAY;}
void DFF34(void){Clear_Bit(SLAVE2, PB, 1); DFF_DELAY; Set_Bit(SLAVE2, PB, 1); DFF_DELAY;}
void DFF35(void){Clear_Bit(SLAVE2, PB, 2); DFF_DELAY; Set_Bit(SLAVE2, PB, 2); DFF_DELAY;}
void DFF36(void){Clear_Bit(SLAVE2, PB, 3); DFF_DELAY; Set_Bit(SLAVE2, PB, 3); DFF_DELAY;}
void DFF37(void){Clear_Bit(SLAVE2, PB, 4); DFF_DELAY; Set_Bit(SLAVE2, PB, 4); DFF_DELAY;}
void DFF38(void){Clear_Bit(SLAVE2, PB, 5); DFF_DELAY; Set_Bit(SLAVE2, PB, 5); DFF_DELAY;}
void DFF39(void){Clear_Bit(SLAVE2, PB, 6); DFF_DELAY; Set_Bit(SLAVE2, PB, 6); DFF_DELAY;}
void DFF40(void){Clear_Bit(SLAVE2, PB, 7); DFF_DELAY; Set_Bit(SLAVE2, PB, 7); DFF_DELAY;}

void DFF_GND_Init(void)
{
	DFF_Data(0x00);

	DFF_DELAY;

	DFF1();
	DFF2();
	DFF3();
	DFF4();
	DFF5();
	DFF6();
	DFF7();
	DFF8();

	DFF9();
	DFF10();
	DFF11();
	DFF12();
	DFF13();
	DFF14();
	DFF15();
	DFF16();

	DFF17();
	DFF18();
	DFF19();
	DFF20();

	DFF21();
	DFF22();
	DFF23();
	DFF24();
	DFF25();
	DFF26();
	DFF27();
	DFF28();

	DFF29();
	DFF30();
	DFF31();
	DFF32();
	DFF33();
	DFF34();
	DFF35();
	DFF36();

	DFF37();
	DFF38();
	DFF39();
	DFF40();
}

void DFF_Floating_Init(void)
{
	DFF_Data(0xFF);

	DFF_DELAY;

	DFF1();
	DFF2();
	DFF3();
	DFF4();
	DFF5();
	DFF6();
	DFF7();
	DFF8();

	DFF9();
	DFF10();
	DFF11();
	DFF12();
	DFF13();
	DFF14();
	DFF15();
	DFF16();

	DFF17();
	DFF18();
	DFF19();
	DFF20();

	DFF21();
	DFF22();
	DFF23();
	DFF24();
	DFF25();
	DFF26();
	DFF27();
	DFF28();

	DFF29();
	DFF30();
	DFF31();
	DFF32();
	DFF33();
	DFF34();
	DFF35();
	DFF36();

	DFF37();
	DFF38();
	DFF39();
	DFF40();
}
