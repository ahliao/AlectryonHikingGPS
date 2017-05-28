// I2C
// by: Alex Liao and Clark Zhang
// date created: 4/26/2014
// Alectryon Technologies
// modified off of code from http://www.embedds.com/programming-avr-i2c-interface/

#include "I2C.hpp"

void I2C::init() {
	// set SCL to 100kHz
	TWSR = 0x00;
	TWBR = 0x20;
	// enable TWI
	TWCR = (1 << TWEN);
}

// send start signal
uint8_t I2C::start() {
	// setting up TWCR register for start signal
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	// waiting for TWINT (interrupt) to be set, signifying
	// the transmission is complete
	while ((TWCR & (1 << TWINT)) == 0);
	// returning status register masking the clock scale bits
	return (TWSR & 0xF8);
}

// send stop signal
uint8_t I2C::stop() {
	// setting up TWCR register for stop
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
	// returning status register masking the clock scale bits
	return (TWSR & 0xF8);
}

// writes a byte of data
uint8_t I2C::write(uint8_t data) {
	// putting data in TWDR register
	TWDR = data;
	TWCR = (1 << TWINT) | (1 << TWEN);
	// waiting for TWINT to be set
	while ((TWCR & (1 << TWINT)) == 0);	
	// returning status reg, masking the clock scale bits
	return (TWSR & 0xF8);
}

// read byte with ACK
uint8_t I2C::readAck() {
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
	while ((TWCR & (1 << TWINT)) == 0);
	return TWDR;
}

//read byte with NACK
uint8_t I2C::readNack() {
	TWCR = (1 << TWINT) | (1 << TWEN);
	while ((TWCR & (1 << TWINT)) == 0);
	return TWDR;
}
