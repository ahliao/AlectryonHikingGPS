// Main program for Trail Utility Tool
// Processor: Atmel atmega328p
// Author: Alex Liao
// Company: Alectryon Technologies
// Date created: 4/27/2014

// Clock speed
#define F_CPU 16000000UL

#define __AVR_ATmega328P__ 1

#include <avr/io.h>
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
#define heartBeatPin 6

// function prototypes
static uint8_t find_file_in_dir(fat_fs_struct* fs, fat_dir_struct* dd, const char* name, fat_dir_entry_struct* dir_entry);
static fat_file_struct* open_file_in_dir(fat_fs_struct* fs, fat_dir_struct* dd, const char* name);
static void loadBMP(fat_fs_struct *fs, fat_dir_struct* dd, const char* file, 
		fat_file_struct *fd, int16_t *width, int16_t *height);
//static void drawBMP24(fat_fs_struct* fs, fat_dir_struct* dd, const char* file);
static void drawBMP16(fat_file_struct *fd, const uint16_t x, const uint16_t y, 
		const int16_t width, const int16_t height);
static void drawPartBMP16(fat_file_struct *fd, const uint16_t x, const uint16_t y, 
		const uint16_t drawWidth, const uint16_t drawHeight, const uint16_t imgX, 
		const uint16_t imgY, const int16_t imgWidth, const int16_t imgHeight);

const uint8_t HEADERSIZE = 70;	// 54 header + 16 color mask
const uint8_t LCDWIDTH = 128;
const uint8_t LCDHEIGHT = 160;
const uint16_t BUFFERSIZE = 128 * 2;
uint8_t buffer[BUFFERSIZE]; // Not really the best way...

int main() {
	// Init communication
	UART::initUART(9600, true);

	// start up led sequence
	DDRD = 0xFE;
	PORTD = 0x00;

	DDRC = 0xFF;
	PORTC = 0x02;

	GPS::GPSData gpsdata;
	uint8_t response = 0;
	uint32_t temp = 0;

	// Init LCD screen
	LCD::initSPI();
	LCD::initDisplay();
	LCD::setOrientation(0);
	LCD::clearDisp();

	PORTC = 0x01;

	// Init SD communication
	if (!sd_raw_init()) {
		// failed MMC/SD init
		PORTC = 0x02;
		LCD::putCh('A',20,50,BLACK);
		PORTC = 0x01;
	}

	partition_struct *partition = partition_open(sd_raw_read,
												 sd_raw_read_interval,
												 0, 0, 0);

	if (!partition)
	{
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

	// directory listing
	/*fat_dir_entry_struct dir_entry;
	uint8_t y = 10;
	while (fat_read_dir(dd, &dir_entry)) {
		//  dir_entry.long_name
		PORTC = 0x02;
		LCD::drawString(dir_entry.long_name, 50, y, BLACK);
		y += 10;
		PORTC = 0x01;
	}*/

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
		PORTC = 0x02;
		LCD::drawString("Error", 50, 70, BLACK);
		PORTC = 0x01;
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

	/*intptr_t count;
	while ((count = fat_read_file(fd, buffer, sizeof(buffer))) > 0)
	{
		for (intptr_t i = 0; i < count; ++i) 
		{
			PORTC = 0x02;
			LCD::drawString(buffer, 8, 50, 60, BLACK);
			PORTC = 0x01;
		}
	}*/
	fat_close_file(fd);

	int16_t imgWidth, imgHeight;
	loadBMP(fs, dd, "ABQ.bmp", fd, &imgWidth, &imgHeight);

	//uint8_t x = 0, velx = 4, vely = 4;
	//y = 40;
	uint16_t imgX, imgY;
	uint8_t drawX, drawY, lastDrawX = 0, lastDrawY = 0;
	//uint8_t velx=4, vely=4, x=0, y=20;
	uint16_t offsetX, offsetY;
	uint16_t lastOffsetX = 0, lastOffsetY = 0;

	bool redraw = false;

	while (1) {
		response = GPS::loadData(gpsdata);

		if (redraw) {
			// Draw the GPS location
			imgX = (uint16_t)((float)abs((gpsdata.latitude - x0) / scaleX));
			imgY = (uint16_t)((float)abs((gpsdata.longitude - y0) / scaleY));
			drawX = (imgX % 64) + 32;
			drawY = (imgY % 80) + 40;

			// Logic on where to draw here
			//offsetX = (imgX / 128) * 128;
			//offsetY = (imgY / 160) * 160;	// Because BMP are bott to top
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
				// Just blink
				drawPartBMP16(fd, lastDrawX-8, lastDrawY-8, 16, 16, 
						offsetX, offsetY, imgWidth, imgHeight);
				_delay_ms(200);
				LCD::fillRect(drawX-4, drawY-4, drawX+4, drawY+4, LIME);
				_delay_ms(200);
			}

			redraw = false;
			lastOffsetX = offsetX;
			lastOffsetY = offsetY;
			lastDrawX = drawX;
			lastDrawY = drawY;
		}

		if (response == GPS::GPGGA) {
			/*UART::writeByte('A');
			// Send the status
			UART::writeByte(gpsdata.status);

			// Send latitude
			memcpy(&temp, &gpsdata.latitude, sizeof(float));
			UART::writeByte(temp >> 24);
			UART::writeByte(temp >> 16);
			UART::writeByte(temp >> 8);
			UART::writeByte(temp);

			// Send longitude
			memcpy(&temp, &gpsdata.longitude, sizeof(float));
			UART::writeByte(temp >> 24);
			UART::writeByte(temp >> 16);
			UART::writeByte(temp >> 8);
			UART::writeByte(temp);

			UART::writeByte('\r');
			UART::writeByte('\n');*/

			redraw = true;
		}
	}
	fat_close_file(fd);	// Close the image
	return 0;
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

	PORTC = 0x02;
	LCD::setOrientation(4);
	LCD::setAddrWindow(0, 0, LCDWIDTH, LCDHEIGHT);
	LCD::writeCmd(RAMWR);
	for (row = 0; row < LCDHEIGHT; ++row) {
		PORTC = 0x01;
		offset = (width-LCDWIDTH)*2;	// Offset to the next row
		fat_read_file(fd, buffer, sizeof(buffer));
		fat_seek_file(fd, &offset, FAT_SEEK_CUR);
		PORTC = 0x02;
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

	PORTC = 0x02;
	LCD::setOrientation(4);
	LCD::setAddrWindow(x, y, x+drawWidth, y+drawHeight);
	LCD::writeCmd(RAMWR);
	for (row = 0; row <= drawHeight; ++row) {
		PORTC = 0x01;
		offset = (imgWidth-LCDWIDTH)*2;	// Offset to the next row
		fat_read_file(fd, buffer, sizeof(buffer));
		fat_seek_file(fd, &offset, FAT_SEEK_CUR);
		PORTC = 0x02;
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

/*void drawBMP24(fat_fs_struct* fs, fat_dir_struct* dd, const char* file)
{
	
	// Read the file
	fat_file_struct *fd = open_file_in_dir(fs, dd, file);
	if (!fd) {
		// Error opening file
	}

	// Read the bmp header
	const uint8_t HEADERSIZE = 54;
	const uint16_t BUFFERSIZE = 128 * 3;
	const uint8_t LCDWIDTH = 128;
	const uint8_t LCDHEIGHT = 160;
	uint8_t buffer[BUFFERSIZE]; // Not really the best way...

	intptr_t count;
	count = fat_read_file(fd, buffer, HEADERSIZE);
	uint32_t width = buffer[18] | (((uint32_t)buffer[19]) << 8) | 
		(((uint32_t)buffer[20]) << 16) | (((uint32_t)buffer[21]) << 24);
	uint32_t height = buffer[22] | (((uint32_t)buffer[23]) << 8) | 
		(((uint32_t)buffer[24]) << 16) | (((uint32_t)buffer[26]) << 24);

	// Now lets get the pixel data
	// Organized bottom to top, left to right
	uint16_t color = 0;
	uint8_t rgb, row, col;
	for (row = 0; row < LCDHEIGHT; ++row) {
		count = fat_read_file(fd, buffer, sizeof(buffer));
		for (col = 0; col < LCDWIDTH; ++col) {
			rgb = buffer[3*(col)];
			color = (((int)((rgb) / 8.0f)&0x001F) << 11);
			rgb = buffer[3*(col) + 1];
			color |= (((uint16_t)((rgb) / 4.0f)&0x003F) << 5);
			rgb = buffer[3*(col) + 2];
			color |= ((uint16_t)(rgb / 8.0f)&0x001F);
			PORTC = 0x02;
			LCD::drawPixel(col, LCDHEIGHT - row - 1, color);
			PORTC = 0x01;
		}
	}

	fat_close_file(fd);
}*/
