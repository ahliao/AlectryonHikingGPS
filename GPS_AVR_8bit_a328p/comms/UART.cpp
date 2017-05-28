// UART low level library (header)
// Changed from class to namespace
// Device: Atmega328p
// By: Alex Liao, Clark Zhang
// Date Created: 6/5/2014
// Alectryon Technologies

#include "UART.hpp"

namespace UART
{
	/**
		Stores the data received by Rx
	*/
	volatile uint8_t UART_data;

	/**
	  	Set to true when new data is received
	*/
	volatile bool UART_newData;

	// TODO: scope hiding (Currently just not declared in header)
	volatile bool UART_can_transmit;

	// Initializes the hardware UART
	void initUART(uint16_t baud, bool enableInterrupt)	{
		// Setting baud rate
		uint16_t baud_rate = F_CPU / (baud * 16UL) - 1;
		UBRR0H = (baud_rate >> 8);
		UBRR0L = baud_rate;

		// Set 8-bit data length with 1-bit stop
		UCSR0C |= (1 << USBS0) | (1 << UCSZ00) | (1 << UCSZ01);

		// Enabling Tx and Rx
		UCSR0B |= (1 << TXEN0) | (1 << RXEN0);

		// If interrupts are used, the readByte() will be slow
		if (enableInterrupt) {
			// Enable receiver interrupt and transmission complete interrupt
			UCSR0B |= (1 << RXCIE0) | (1 << TXCIE0);

			// TODO: Should we put this in here?
			// Enables interrupts
			sei();
		}
	}

	void writeByte(uint8_t data) {
		// If the USART Data Register (buffer) is empty
		while (!(UCSR0A & (1 << UDRE0))) {};
		UDR0 = data;
		UART_can_transmit = false;	// TODO: Use this?
	}

	uint8_t readByte() {
		while (!(UCSR0A & (1 << RXC0))) {};
		return UDR0;
	}
};

// Interrupt functions for the UART hardware

/** 
  	Interrupt for data received
*/
ISR(USART_RX_vect) {
	UART::UART_data = UDR0;
	UART::UART_newData = true;
}

/** 
  	Interrupt for complete transmission
*/
ISR(USART_TX_vect) {
	UART::UART_can_transmit = true;
}
