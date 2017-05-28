// UART low level library (header)
// Device: Atmega328p
// By: Alex Liao, Clark Zhang
// Date Created: 6/5/2014
// Alectryon Technologies

#ifndef _UART_
#define _UART_

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

/**
	Namespace for UART related functions
*/
namespace UART
{
	/**
	  	Initializes the hardware UART

		This is a low level function to initialize UART in the AVR MCU. 
		Currently set to work with the Atmega328p
		Default baud set at 9600. Needs to have F_CPU macro defined.
		If UART interrupts are to be used, set enableInterrupt to 1 (true)
		@param baud Baud to communicate at
		@param enableInterrupt Set to use interrupts with UART
		@return none
	*/
	void initUART(uint16_t baud = 9600, bool enableInterrupt = false);
 
	/** 
	  	Writes a byte over Tx 

		This is a low level function to send a byte of data over the Tx wire. 
		Any confirmation signal goes through Rx
		@param data single byte (8 bits) of data to be sent
		@return none
	*/
	void writeByte(uint8_t data);

	/** 
	  	Waits for a new byte to be received

		This is a low level function to read a byte of data over the Rx wire.
		Use this when interrupts are not needed 
	*/
	uint8_t readByte();

	/**
		Stores the data received by Rx
	*/
	extern volatile uint8_t UART_data;

	/**
	  	Set to true when new data is received
	*/
	extern volatile bool UART_newData;
};

#endif
