/****************************************
* "I2C.h":                              *
* Header file for Mega2560 I2C driver.  *
* Also compatible with Mega16/Mega32.   * 
*                                       *
*****************************************/
void i2c_init();
void i2c_start();
void i2c_write(unsigned char data);
unsigned char i2c_read (unsigned char isLast);
void i2c_stop();
/****************************************/