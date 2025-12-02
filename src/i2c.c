/************************************************
* "I2C.c":                                      *
* Implementation file for Mega2560 I2C driver.  *
* Also compatible with Mega16/Mega32.           * 
*                                               *
*************************************************/
#include <avr/io.h>

void i2c_init()
{
  // TWI prescaler = 1 (same as default)
  TWSR = 0;
  // LM75 clock has be to be lower than 400 kHz (according to LM75 data sheet)
  // We choose 2-wire clock ~ 216 kHz (when fCPU = 16 MHz)
  // The TWBR must be at least 10 in master mode (AVR data book)
  // SCL frequency = fCPU / (16 + 2*TWBR*1), when prescaler = 1
  TWBR = 29;
}

void i2c_start()
{
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
  while ((TWCR & (1<<TWINT)) == 0)
  {}
}

void i2c_write(unsigned char data)
{
  TWDR = data;
  TWCR = (1<<TWINT) | (1<<TWEN);
  while ((TWCR & (1<<TWINT)) == 0)
  {}
}

unsigned char i2c_read (unsigned char isLast)
{
  // If this is not the last byte to read
  if (isLast == 0)
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
  // If this is the last byte to read 	
  else
    TWCR = (1<<TWINT) | (1<<TWEN);
  while ((TWCR & (1<<TWINT)) == 0)
  {}
  return TWDR;
}

void i2c_stop()
{
  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
}

