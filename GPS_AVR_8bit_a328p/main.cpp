// Main program for Trail Utility Tool
// Processor: Atmel atmega328p
// Author: Alex Liao
// Company: Alectryon Technologies
// Date created: 4/27/2014

// Clock speed
#define F_CPU 16000000UL

#define __AVR_ATmega328P__ 1

#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>

#include "comms/UART.hpp"
#include "common/GPSData.hpp"
#include "drivers/ST7735.hpp"

#include "drivers/fat.h"
#include "drivers/fat_config.h"
#include "drivers/partition.h"
#include "drivers/sd_raw.h"
#include "drivers/sd_raw_config.h"

// Define pin outputs
#define LCD_CS 2
#define selectLCD() PORTB &= ~(1 << LCD_CS)
#define unselectLCD() PORTB |= (1 << LCD_CS)
// Status LEDs on PORTC0 and PORTC1
#define STATUS1 0
#define STATUS2 1
#define MENUBTN 2
#define UPBTN 3
#define DOWNBTN 4

// Operation Modes
#define MODE_MAP 0
#define MODE_MENU 1
#define MODE_SCROLL 2

// function prototypes
static uint8_t find_file_in_dir(fat_fs_struct* fs, fat_dir_struct* dd, 
		const char* name, fat_dir_entry_struct* dir_entry);
static fat_file_struct* open_file_in_dir(fat_fs_struct* fs, 
		fat_dir_struct* dd, const char* name);
static void loadBMP(fat_fs_struct *fs, fat_dir_struct* dd, const char* file, 
		fat_file_struct *fd, int16_t *width, int16_t *height);
static void drawBMP16(fat_file_struct *fd, const uint16_t x, 
		const uint16_t y, const int16_t width, const int16_t height);
static void drawPartBMP16(fat_file_struct *fd, 
		const uint16_t x, const uint16_t y, const uint16_t drawWidth, 
		const uint16_t drawHeight, const uint16_t imgX, 
		const uint16_t imgY, const int16_t imgWidth, const int16_t imgHeight);
static void handleInputs();
static void lowPowerMode();
static void normalMode();

const uint8_t HEADERSIZE = 70;	// 54 header + 16 color mask
const uint8_t LCDWIDTH = 128;
const uint8_t LCDHEIGHT = 160;
const uint16_t BUFFERSIZE = 128 * 2;
uint8_t buffer[BUFFERSIZE]; // Not really the best way...

// Global Variables
GPS::GPSData gpsdata;
uint16_t imgX, imgY;
uint8_t drawX, drawY, lastDrawX = 0, lastDrawY = 0;
uint16_t offsetX, offsetY;
uint16_t lastOffsetX = 0, lastOffsetY = 0;

// Timer2 overflow counter
uint8_t overflowCounter2 = 0;

// Current mode of the tool
uint8_t currentMode = MODE_MAP;

uint8_t menuSelection = 0;
const uint8_t menuSize = 5;
const char* menuItems[menuSize] = {"Map View", "Load Trail", "Settings", "About", "Sleep"};

// Temp testing stuff
float latitude = 0, longitude = 0, step = 0;
bool redraw = false;

int main() {
	// Turn on interrupt for power on
	/*DDRD &= ~(1 << DDD2);
	PORTD |= (1 << PORTD2);
	EICRA |= (1 << ISC01);
	EIMSK |= (1 << INT0);	// Turns on INT0*/

	// Init communication
	UART::initUART(9600, true);

	// Setup LED Ports as output
	DDRC |= (1 << DDC0) | (1 << DDC1);

	// Startup LED Sequence
	PORTC |= (1 << STATUS1);
	_delay_ms(250);
	PORTC ^= (1 << STATUS1) | (1 << STATUS2);
	_delay_ms(250);
	PORTC ^= (1 << STATUS1) | (1 << STATUS2);
	_delay_ms(250);
	PORTC = 0x00;

	// Init basic variables
	uint8_t response = 0;
	uint32_t temp = 0;

	// Init LCD screen
	LCD::initSPI();
	DDRB |= (1 << DDB2);	// LCD CS, SD card handles the PORTD7
	selectLCD();
	LCD::initDisplay();
	LCD::setOrientation(0);
	LCD::clearDisp();

	// Setup PWM for LCD brightness
	TCCR0A |= (1 << WGM00) | (1 << WGM01) | (1 << COM0A1);
	TCCR0B |= (1 << CS01);
	DDRD |= (1 << DDD6); // This is on PD6
	OCR0A = 128;

	// Timer 2 for general timing without using delays
	//TCCR2A |= (1 << WGM20) | (1 << WGM21);
	TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);
	TIMSK2 |= (1 << TOIE2);

	// Setup input up and down buttons
	DDRD &= ~((1 << MENUBTN) | (1 << UPBTN) | (1 << DOWNBTN));
	PORTD |= (1 << MENUBTN) | (1 << UPBTN) | (1 << DOWNBTN);

	unselectLCD();	// SD Drivers handles the chip select for the SD Card

	// Init SD communication
	if (!sd_raw_init()) {
		// failed MMC/SD init
	}

	partition_struct *partition = partition_open(sd_raw_read,
												 sd_raw_read_interval,
												 0, 0, 0);

	if (!partition)	{
		// Failed to open partition
	}

	// open file system
	fat_fs_struct *fs = fat_open(partition);
	if (!fs) {
		// failed to open filesystem
	}

	// open root directory
	fat_dir_entry_struct directory;
	fat_get_dir_entry_of_path(fs, "/", &directory);
	fat_dir_struct *dd = fat_open_dir(fs, &directory);
	if (!dd) {
		// failed to open root directory
	}

	// Read the trail file
	// This file holds the geo-referencing, waypoint data
	// Format: 
	// 4 bytes = lat of x0
	// 4 bytes = long of y0
	// 4 bytes = scale X (lat per pixel)
	// 4 bytes = scale Y (long per pixel)
	fat_file_struct *fd = open_file_in_dir(fs, dd, "ABQ.trail");
	if (!fd) {
		// Error opening file
		selectLCD();
		LCD::drawString("Error", 50, 70, BLACK);
		unselectLCD();
	}
	uint8_t buffer[16];
	fat_read_file(fd, buffer, sizeof(buffer));
	float x0, y0, scaleX, scaleY;
	temp = (uint32_t)buffer[0] | (uint32_t)buffer[1] << 8 
		| (uint32_t)buffer[2] << 16 | (uint32_t)buffer[3] << 24;
	memcpy(&x0, &temp, sizeof(x0));
	temp = (uint32_t)buffer[4] | (uint32_t)buffer[5] << 8 
		| (uint32_t)buffer[6] << 16 | (uint32_t)buffer[7] << 24;
	memcpy(&y0, &temp, sizeof(y0));
	temp = (uint32_t)buffer[8] | (uint32_t)buffer[9] << 8 
		| (uint32_t)buffer[10] << 16 | (uint32_t)buffer[11] << 24;
	memcpy(&scaleX, &temp, sizeof(scaleX));
	temp = (uint32_t)buffer[12] | (uint32_t)buffer[13] << 8 
		| (uint32_t)buffer[14] << 16 | (uint32_t)buffer[15] << 24;
	memcpy(&scaleY, &temp, sizeof(scaleY));

	fat_close_file(fd);

	// Load the BMP image
	int16_t imgWidth, imgHeight;
	selectLCD();
	loadBMP(fs, dd, "ABQ.bmp", fd, &imgWidth, &imgHeight);

	while (1) {
		response = GPS::loadData(gpsdata);

		switch (currentMode) {
			case MODE_MAP:
				if (redraw) {
					// Draw the GPS location
					imgX = (uint16_t)((float)abs((gpsdata.latitude - x0) / scaleX));
					imgY = (uint16_t)((float)abs((gpsdata.longitude - y0) / scaleY));
					drawX = (imgX % 64) + 32;
					drawY = (imgY % 80) + 40;

					// Logic on where to draw here
					offsetX = (imgX / 64) * 64 - 32;
					offsetY = (imgY / 80) * 80 - 40;	// Because BMP are bott to top
					if (lastOffsetX != offsetX || lastOffsetY != offsetY) {
						drawBMP16(fd, offsetX, offsetY, imgWidth, imgHeight);
						LCD::fillRect(drawX-4, drawY-4, drawX+4, drawY+4, LIME);
					} else {
						// Draw the lat and long text
						LCD::setOrientation(0);
						LCD::drawString(gpsdata.latStr, 5, 5, BLACK);
						LCD::drawString(gpsdata.longStr, 5, 15, BLACK);
						if (abs(lastDrawX - drawX) >= 1 || abs(lastDrawY - drawY) >= 1) {
							// Just blink the square
							drawPartBMP16(fd, lastDrawX-8, lastDrawY-8, 16, 16, 
									lastOffsetX, lastOffsetY, imgWidth, imgHeight);
							LCD::fillRect(drawX-4, drawY-4, drawX+4, drawY+4, LIME);
						}
					}

					redraw = false;
					lastOffsetX = offsetX;
					lastOffsetY = offsetY;
					lastDrawX = drawX;
					lastDrawY = drawY;
				}

				if (response == GPS::GPGGA) {
					redraw = true;
					if (latitude == 0) {
						latitude = gpsdata.latitude;
						longitude = gpsdata.longitude;
					}
				}

				break;
			case MODE_MENU:
				if (redraw) {
					LCD::fillRect(0, 0, 16, 160, WHITE);
					// Draw the menu cursor
					LCD::fillRect(11, 26 + menuSelection * 10, 16, 31 + menuSelection * 10, RED);
					redraw = false;
					//_delay_ms(200);
				}
				break;
			case MODE_SCROLL:
				if (redraw) {
					// Draw the GPS location
					imgX = (uint16_t)((float)abs((latitude - x0) / scaleX));
					imgY = (uint16_t)((float)abs((longitude - y0) / scaleY));
					drawX = (imgX % 64) + 32;
					drawY = (imgY % 80) + 40;

					// Logic on where to draw here
					offsetX = (imgX / 64) * 64 - 32;
					offsetY = (imgY / 80) * 80 - 40;	// Because BMP are bott to top
					if (lastOffsetX != offsetX || lastOffsetY != offsetY) {
						drawBMP16(fd, offsetX, offsetY, imgWidth, imgHeight);
						LCD::fillRect(drawX-4, drawY-4, drawX+4, drawY+4, LIME);
					} else {
						// Draw the lat and long text
						/*LCD::setOrientation(0);
						LCD::drawString(gpsdata.latStr, 5, 5, BLACK);
						LCD::drawString(gpsdata.longStr, 5, 15, BLACK);*/
						if (abs(lastDrawX - drawX) >= 1 || abs(lastDrawY - drawY) >= 1) {
							// Just blink the square
							drawPartBMP16(fd, lastDrawX-8, lastDrawY-8, 16, 16, 
									lastOffsetX, lastOffsetY, imgWidth, imgHeight);
							LCD::fillRect(drawX-4, drawY-4, drawX+4, drawY+4, LIME);
						}
					}

					redraw = false;
					lastOffsetX = offsetX;
					lastOffsetY = offsetY;
					lastDrawX = drawX;
					lastDrawY = drawY;
				}
				break;
			default:
				break;
		}
		// Handle any button inputs 
		if (overflowCounter2 > 10) {
			handleInputs();
		}
	}
	fat_close_file(fd);	// Close the image
	return 0;
}

void handleInputs() {
	// Handle button inputs
	if (!(PIND & (1 << MENUBTN))) {
		switch (currentMode) {
			case MODE_MAP:
				// Display a menu
				LCD::setOrientation(0);
				LCD::clearDisp();
				LCD::drawString("Menu", 25, 10, BLACK);
				for (uint8_t i = 0; i < menuSize; ++i) {
					LCD::drawString(menuItems[i], 25, 25 + i * 10, BLACK);
				}
				menuSelection = 0;
				currentMode = MODE_MENU;
				redraw = true;
				break;
			case MODE_MENU:
				// If going into SCROLL MODE
				//latitude = gpsdata.latitude;
				//longitude = gpsdata.longitude;
				currentMode = MODE_MAP;
				lastOffsetX = 0;
				lastOffsetY = 0;
				redraw = true;
				break;
			case MODE_SCROLL:
				step = -0.0010;
				break;
			default:
				break;
		}
		overflowCounter2 = 0;
	} else {
		switch (currentMode) {
			case MODE_MAP:
				break;
			case MODE_MENU:
				break;
			case MODE_SCROLL:
				break;
			default:
				break;
		}
	}
	if (!(PIND & (1 << UPBTN))) {
		switch (currentMode) {
			case MODE_MAP:
				break;
			case MODE_MENU:
				if (menuSelection > 0) --menuSelection;
				redraw = true;
				break;
			case MODE_SCROLL:
				PORTC |= (1 << STATUS1);
				latitude += step;
				redraw = true;
				_delay_ms(200);
				break;
			default:
				break;
		}
		overflowCounter2 = 0;
	} else {
		switch (currentMode) {
			case MODE_MAP:
				break;
			case MODE_MENU:
				break;
			case MODE_SCROLL:
				break;
			default:
				break;
		}
		PORTC &= ~(1 << STATUS1);
	}
	if (!(PIND & (1 << DOWNBTN))) {
		switch (currentMode) {
			case MODE_MAP:
				break;
			case MODE_MENU:
				if (menuSelection < menuSize - 1) ++menuSelection;
				redraw = true;
				break;
			case MODE_SCROLL:
				PORTC |= (1 << STATUS2);
				longitude += step;
				redraw = true;
				_delay_ms(200);
				break;
			default:
				break;
		}
		overflowCounter2 = 0;
	} else {
		switch (currentMode) {
			case MODE_MAP:
				break;
			case MODE_MENU:
				break;
			case MODE_SCROLL:
				break;
			default:
				break;
		}
		PORTC &= ~(1 << STATUS2);
	}
}

// This requires width to be a multiple of LCDWIDTH atm..
void drawBMP16(fat_file_struct *fd, const uint16_t x, const uint16_t y, 
		const int16_t width, const int16_t height)
{
	// Now lets get the pixel data
	// Organized bottom to top, left to right
	int32_t offset=(int32_t)(width)*y*2+x*2 + HEADERSIZE;	
	uint16_t color;
	uint8_t row, col, temp;

	fat_seek_file(fd, &offset, FAT_SEEK_SET);

	selectLCD();
	LCD::setOrientation(4);
	LCD::setAddrWindow(0, 0, LCDWIDTH, LCDHEIGHT);
	LCD::writeCmd(RAMWR);
	for (row = 0; row < LCDHEIGHT; ++row) {
		unselectLCD();
		offset = (width-LCDWIDTH)*2;	// Offset to the next row
		fat_read_file(fd, buffer, sizeof(buffer));
		fat_seek_file(fd, &offset, FAT_SEEK_CUR);
		selectLCD();
		for (col = 0; col < LCDWIDTH; ++col) {
			temp = col << 1;	// Divide by 2
			color = buffer[temp];
			color |= buffer[temp+1]<<8;
			LCD::sendSPI(color >> 8);
			LCD::sendSPI(color);
		}
	}
}

void drawPartBMP16(fat_file_struct *fd, const uint16_t x, const uint16_t y, 
		const uint16_t drawWidth, const uint16_t drawHeight, const uint16_t imgX, 
		const uint16_t imgY, const int16_t imgWidth, const int16_t imgHeight)
{
	// Now lets get the pixel data
	// Organized bottom to top, left to right
	int32_t offset=(int32_t)(imgWidth)*(imgY+y)*2+(imgX+x)*2 + HEADERSIZE;	
	uint16_t color;
	uint8_t row, col, temp;

	fat_seek_file(fd, &offset, FAT_SEEK_SET);

	selectLCD();
	LCD::setOrientation(4);
	LCD::setAddrWindow(x, y, x+drawWidth, y+drawHeight);
	LCD::writeCmd(RAMWR);
	for (row = 0; row <= drawHeight; ++row) {
		unselectLCD();
		offset = (imgWidth-LCDWIDTH)*2;	// Offset to the next row
		fat_read_file(fd, buffer, sizeof(buffer));
		fat_seek_file(fd, &offset, FAT_SEEK_CUR);
		selectLCD();
		for (col = 0; col <= drawWidth; ++col) {
			temp = col << 1;	// Divide by 2
			color = buffer[temp];
			color |= buffer[temp+1]<<8;
			LCD::sendSPI(color >> 8);
			LCD::sendSPI(color);
		}
	}
}

void loadBMP(fat_fs_struct *fs, fat_dir_struct* dd, const char* file, fat_file_struct *fd, 
		int16_t *width, int16_t *height)
{
	// Read the file
	fd = open_file_in_dir(fs, dd, file);
	if (!fd) {
		// Error opening file
	}

	// Read the bmp header
	fat_read_file(fd, buffer, HEADERSIZE);
	// width and height will likely never need 32-bits
	*width = buffer[18] | (((uint16_t)buffer[19]) << 8);
	*height = buffer[22] | (((uint16_t)buffer[23]) << 8);
}

uint8_t find_file_in_dir(fat_fs_struct* fs, fat_dir_struct* dd,
		const char* name, fat_dir_entry_struct* dir_entry)
{
	while(fat_read_dir(dd, dir_entry))
	{
		if (strcmp(dir_entry->long_name, name) == 0)
		{
			fat_reset_dir(dd);
			return 1;
		}
	}
	return 0;
}

fat_file_struct* open_file_in_dir(fat_fs_struct* fs, 
		fat_dir_struct* dd, const char* name)
{
	fat_dir_entry_struct file_entry;
	if(!find_file_in_dir(fs, dd, name, &file_entry))
		return 0;
	return fat_open_file(fs, &file_entry);
}

void lowPowerMode()
{
	selectLCD();
	// Set the LCD to sleep mode
	LCD::writeCmd(SLPIN);
	TCCR0A = 0;
	TCCR0B = 0;	// Turn off the PWM

	// Set the GPS to low power mode
	GPS::enterStandby();
}

void normalMode()
{
	selectLCD();
	// Turn on LCD and PWM backlight
	LCD::writeCmd(SLPOUT);
	TCCR0A |= (1 << WGM00) | (1 << WGM01) | (1 << COM0A1);
	TCCR0B |= (1 << CS01);

	GPS::wakeup();
}

ISR (INT0_vect)
{
	/*
	cli();
	// Turn on the processor
	lowPowerMode();
	_delay_ms(1500);	// button bouncing

	// Set the AVR to low power
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_enable();
	sei();
	sleep_cpu();
	sleep_disable();	// Where program continues
	normalMode();

	_delay_ms(1500);	// button bouncing
	*/
}

ISR (TIMER2_OVF_vect)
{
	++overflowCounter2;
}
