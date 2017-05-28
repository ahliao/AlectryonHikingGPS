// I2C
// by: Alex Liao and Clark Zhang
// date created: 4/26/2014
// Alectryon Technologies
// modified off of code from http://www.embedds.com/programming-avr-i2c-interface/

#ifndef _I2C_
#define _I2C_

#include <avr/io.h>
#include <util/twi.h>
#include <stdint.h>

/**
	Low level i2c support library. Does low level i2c transactions.
*/
class I2C {
public:
	/**
	Initializes i2c communication

	Sets up SCL rate at 100kHz (assumming clock speed is 8MHz) and enables TWI. Modifies the TWI registers.
	@return none
	*/
	static void init();

	/**
	Sets up and sends start condition for i2c communication.

	Function will wait until transmission of start condition is complete
	and the TWINT(interrupt) bit is set.
	@return status code (TWSR & 0xF8)
	*/
	static uint8_t start();

	/**
	Sets up and sends stop condition for i2c communication

	Unlike i2cStart, stop does not wait for an interrupt to be set 
	(there is no interrupt for the stop condition)
	@return status code (should be TW_START if no error occurs)
	*/
	static uint8_t stop();

	/**
	Sets up and sends a single byte of data accross SDA for i2c communication

	This is a low level function to facilitate sending of data inside of the protocol.
	Do not use this to send an actual byte of data that needs to be read by an i2c slave
	To use this, i2cStart() must have been initiated
	@param data data byte to write
	@return status code
	*/
	static uint8_t write(uint8_t data);

	/**
	Sets up and reads a single byte of data accross SDA for i2c communication

	This is a low level function to facilitate reading of data inside of the protocol.
	Do not use this to read an actual byte of data from an i2c slave.
	An acknowledge will also be sent accross to slave after data is read.
	@return byte of data read
	*/
	static uint8_t readAck();

	/**
	Sets up and reads a single byte of data accross SDA for i2c communication

	This is a low level function to facilitate reading of data inside of the protocol.
	Do not use this to read an actual byte of data from an i2c slave.
	A not acknowledge will also be sent accross to slave after data is read.
	Used for reading the last byte of data in an i2c transaction
	@return byte of data read
	*/
	static uint8_t readNack();

	/**
	returns the current status of the TWSR register masking out the bottom config bits

	@return status code
	*/
	static inline uint8_t status() { return TWSR & 0xF8; }
};

#endif
