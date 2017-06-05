#include "ST7735.hpp"
#include <string.h>

#define clearBit(x,y) x &= ~_BV(y)
#define setBit(x,y) x |= _BV(y)

namespace LCD
{
	void initSPI() 
	{
		// Set MOSI and SCK output
		DDRB = 0x2F;
		// Enable SPI, Master, set clock rate fck/4
		// This stuff gets rewritten in the SD Card init
		SPCR = (1 << SPE) | (1 << MSTR);
	}

	void stopSPI()
	{
		SPCR = 0x00;
	}

	uint8_t sendSPI(uint8_t data)
	{
		SPDR = data;	// starts the SPI transfer
		while (!(SPSR & (1 << SPIF)));	// From for complete
		return SPDR;
	}

	void writeCmd(uint8_t cmd)
	{
		clearBit(PORTB, 1);
		sendSPI(cmd);
		setBit(PORTB, 1);
	}

	void hardwareReset()
	{
		clearBit(PORTB, 0);	// Pull PB0 low
		_delay_ms(1);		// pause for 1ms
		setBit(PORTB, 0);	// Pull PB0 high
		_delay_ms(200);		// pause for 200ms to confirm reset
	}

	void initDisplay()
	{
		hardwareReset();	// Reset the display
		writeCmd(SLPOUT);	// Display out of sleep mode
		_delay_ms(150);
		writeCmd(COLMOD);	// Set the color mode
		sendSPI(0x05);		// to mode 5 RGB565 2 bytes
		writeCmd(DISPON);	// turn on display
	}

	void write565(uint16_t data, uint16_t count)
	{
		writeCmd(RAMWR);
		for (; count > 0; --count)
		{
			SPDR = (data >> 8);
			while (!(SPSR & 0x80));
			SPDR = (data & 0xFF);
			while (!(SPSR & 0x80));
		}
	}

	void setAddrWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
	{
		writeCmd(CASET);
		sendSPI(0);
		sendSPI(x0);
		sendSPI(0);
		sendSPI(x1);
		writeCmd(RASET);
		sendSPI(0);
		sendSPI(y0);
		sendSPI(0);
		sendSPI(y1);
	}

	void drawPixel(uint8_t x, uint8_t y, uint16_t color)
	{
		setAddrWindow(x, y, x, y);
		write565(color, 1);
	}

	void fillRect(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint16_t color)
	{
		uint8_t width = x1-x0+1;
		uint8_t height = y1-y0+1;
		setAddrWindow(x0, y0, x1, y1);
		write565(color, width*height);
	}

	void HLine(uint8_t x0, uint8_t x1, uint8_t y, uint16_t color)
	{
		uint8_t width = x1-x0+1;
		setAddrWindow(x0, y, x1, y);
		write565(color, width);
	}

	void VLine(uint8_t x, uint8_t y0, uint8_t y1, uint16_t color)
	{
		uint8_t height = y1-y0+1;
		setAddrWindow(x, y0, x, y1);
		write565(color, height);
	}

	void drawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint16_t color) 
	{
		int dx = abs(x1-x0), sx = x0<x1 ? 1 : -1;
		int dy = abs(y1-y0), sy = y0<y1 ? 1 : -1;
		int err = (dx>dy ? dx : -dy)/2, e2;
		for(;;) {
			drawPixel(x0, y0, color);
			if (x0==x1 && y0==y1) break;
			e2 = err;
			if (e2 > -dx) { err -= dy; x0 += sx; }
			if (e2 < dy) { err += dx; y0 += sy; }
		}
	}

	void putCh(char ch, uint8_t x, uint8_t y, uint16_t color)
	{
		// Write a character is 6x7
		int pixel;
		uint8_t row, col, bit, data, mask = 0x01;
		setAddrWindow(x, y, x+4, y+6);
		writeCmd(RAMWR);

		for (row = 0; row < 7; ++row) 
		{
			for (col = 0; col < 5; ++col)
			{
				data = pgm_read_byte(&(FONT_CHARS[ch-32][col]));
				bit = data & mask;
				if (bit == 0) pixel=WHITE;
				else pixel = color;
				sendSPI(pixel >> 8);
				sendSPI(pixel);
			}
			mask <<= 1;
		}
	}

	void drawString(const char *str, uint8_t x, uint8_t y, uint16_t color)
	{
		uint8_t xt = x;
		for (uint8_t i = 0; i < strlen(str); ++i) {
			putCh(str[i], xt, y, color);
			xt += 6;
		}
	}

	void drawString(const uint8_t *str, uint8_t bufferSize, uint8_t x, uint8_t y, uint16_t color)
	{
		uint8_t xt = x;
		for (uint8_t i = 0; i < bufferSize-1; ++i) {
			putCh((char)str[i], xt, y, color);
			xt += 6;
		}
	}

	void clearDisp()
	{
		fillRect(0, 0, XMAX, YMAX, WHITE);
	}

	void setOrientation(uint8_t degrees)
	{
		uint8_t arg;
		switch (degrees)
		{
			case ORIENTATION_0: arg = 0x00; break;
			case ORIENTATION_90: arg = 0x60; break;
			case ORIENTATION_180: arg = 0xC0; break;
			case ORIENTATION_270: arg = 0xA0; break;
			case ORIENTATION_MY: arg = 0x80; break;	// BMP mirror Y-axis
			case ORIENTATION_MX: arg = 0x20; break;	// BMP mirror Y-axis
			default: arg = 0x00; break;
		}
		writeCmd(MADCTL);
		sendSPI(arg);
	}
}
