// Main program for Trail Utility Tool
// Processor: Atmel atmega328p
// Author: Alex Liao
// Company: Alectryon Technologies
// Date created: 4/27/2017

// Clock speed
#define F_CPU 16000000UL

#define __AVR_ATmega328P__ 1

#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
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
#define UPBTN 4
#define DOWNBTN 3

// function prototypes
static uint8_t find_file_in_dir(fat_fs_struct* fs, fat_dir_struct* dd, 
		const char* name, fat_dir_entry_struct* dir_entry);
static fat_file_struct* open_file_in_dir(fat_fs_struct* fs, 
		fat_dir_struct* dd, const char* name);
static void loadBMP(fat_fs_struct *fs, fat_dir_struct* dd, const char* file, 
		fat_file_struct *fd, int16_t *width, int16_t *height);
static void readMasterFile(fat_fs_struct *fs, fat_dir_struct* dd, fat_file_struct *fd);
static void drawBMP16(fat_file_struct *fd, const uint16_t x, 
		const uint16_t y, const int16_t width, const int16_t height);
static void drawPartBMP16(fat_file_struct *fd, 
		const uint16_t x, const uint16_t y, const uint16_t drawWidth, 
		const uint16_t drawHeight, const uint16_t imgX, 
		const uint16_t imgY, const int16_t imgWidth, const int16_t imgHeight);
static void drawMenu();
static void handleInputs();
static void lowPowerMode();
static void normalMode();
static void drawAlectryonLogo(const uint8_t x, const uint8_t y);

#define HEADERSIZE 70
#define LCDWIDTH 128
#define LCDHEIGHT 160
#define BUFFERSIZE 256
/*const uint8_t HEADERSIZE = 70;	// 54 header + 16 color mask
const uint8_t LCDWIDTH = 128;
const uint8_t LCDHEIGHT = 160;
const uint16_t BUFFERSIZE = 128 * 2;*/
uint8_t buffer[BUFFERSIZE]; // Not really the best way...
char strbuffer[24];

// GPS/Map view variables
GPS::GPSData gpsdata;
int16_t imgWidth, imgHeight;
uint16_t imgX, imgY;
uint8_t drawX, drawY, lastDrawX = 0, lastDrawY = 0;
uint16_t offsetX, offsetY;
uint16_t lastOffsetX = 0, lastOffsetY = 0;
float latitude = 0, longitude = 0, step = 0;
bool redraw = false;

// Files to open
const char *MAPFILE = "Sandia.bmp";
const char *MAPGEOFILE = "Sandia.trail";

// Timer2 overflow counter
uint8_t overflowCounter2 = 0;

// Menu Variables
int8_t menuSelection = 0;
const uint8_t menuSize = 6;
const char menuItem0[] PROGMEM = "Map View";
const char menuItem1[] PROGMEM = "Scroll View";
const char menuItem2[] PROGMEM = "SD Card";
const char menuItem3[] PROGMEM = "Settings";
const char menuItem4[] PROGMEM = "About";
const char menuItem5[] PROGMEM = "Sleep";
PGM_P const menuItems[] PROGMEM = 
{
	menuItem0,
	menuItem1,
	menuItem2,
	menuItem3,
	menuItem4,
	menuItem5
};
uint8_t inputCounter = 0;
// Operation Modes
#define MODE_MENU -1
#define MODE_MAPVIEW 0
#define MODE_SCROLLVIEW 1
#define MODE_SDCARD 2
#define MODE_SETTINGS 3
#define MODE_ABOUT 4
#define MODE_SLEEP 5

// Setting Variables
int8_t settingsSelection = 0;
const uint8_t settingsSize = 5;
const char settingItem0[] PROGMEM = "LCD Brightness";
const char settingItem1[] PROGMEM = "GPS Update Rate";
const char settingItem2[] PROGMEM = "UI Display";
const char settingItem3[] PROGMEM = "Save & Exit";
const char settingItem4[] PROGMEM = "Exit";
PGM_P const settingsItems[] PROGMEM =
{
	settingItem0,
	settingItem1,
	settingItem2,
	settingItem3,
	settingItem4
};
// Selection options
#define SETTINGS_BRIGHTNESS 0
#define SETTINGS_GPSRATE 1
#define SETTINGS_UI 2
#define SETTINGS_SAVE 3
#define SETTINGS_EXIT 4
// Selection mode
#define SETTINGS_MODE_MENU 0
#define SETTINGS_MODE_BRIGHTNESS 1
#define SETTINGS_MODE_GPSRATE 2
#define SETTINGS_MODE_UI 3
// Setting edit variables
uint8_t currentSettingSelection = 0;
uint8_t currentEditPlace = 0;
#define BRIGHTNESS_DIGITS 3
#define GPSRATE_DIGITS 5

// Input counter for hold operations
#define INPUT_TIMER_LIMIT 10

// Firmware and Hardware versions
#define FIRMWARE_VER "0.8"
#define HARDWARE_VER "1.0"
const char ABOUT_0[] PROGMEM = "Alectryon";
const char ABOUT_1[] PROGMEM = "Technologies";
const char ABOUT_2[] PROGMEM = "Firmware Version:";
const char ABOUT_3[] PROGMEM = "Hardware Version:";
const char ABOUT_4[] PROGMEM = "Created By:";
const char ABOUT_5[] PROGMEM = "Alex Liao";
const char ABOUT_6[] PROGMEM = "Clark Zhang";

// Current mode of the tool
int8_t currentMode = MODE_MAPVIEW;

// SD Card variables
partition_struct *partition;
fat_fs_struct *fs;
fat_dir_entry_struct directory;
fat_dir_struct *dd;
fat_file_struct *fd;

// EEPROM variables
uint8_t EEMEM EEPROM_LCD_Brightness = 128;
uint16_t EEMEM EEPROM_GPS_Rate = 1000;
uint8_t EEMEM EEPROM_UI_Mode = 0;

// SRAM Variables from EEPROM
uint8_t LCD_Brightness;
uint16_t GPS_Rate;
uint8_t UI_Mode;
#define NUM_UI_MODES 2
const char UIModeItem0[] PROGMEM = "Minimal ";
const char UIModeItem1[] PROGMEM = "GPS Info";
PGM_P const uiModeItems[] PROGMEM =
{
	UIModeItem0,
	UIModeItem1
};

int main() {
	// Init the gpsdata
	gpsdata.numsats = 0;
	gpsdata.altitude = 0;
	gpsdata.status = GPS_NOFIX;
	// Read settings from the EEPROM
	LCD_Brightness = eeprom_read_byte(&EEPROM_LCD_Brightness);
	GPS_Rate = eeprom_read_word(&EEPROM_GPS_Rate);
	UI_Mode = eeprom_read_byte(&EEPROM_UI_Mode);

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

	partition = partition_open(sd_raw_read,
												 sd_raw_read_interval,
												 0, 0, 0);

	if (!partition)	{
		// Failed to open partition
	}

	// open file system
	fs = fat_open(partition);
	if (!fs) {
		// failed to open filesystem
	}

	// open root directory
	fat_get_dir_entry_of_path(fs, "/", &directory);
	dd = fat_open_dir(fs, &directory);
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
	fd = open_file_in_dir(fs, dd, MAPGEOFILE);
	if (!fd) {
		// Error opening file
		selectLCD();
		LCD::drawString("Error", 50, 70, BLACK);
		unselectLCD();
	}
	uint8_t scalebuffer[16];
	fat_read_file(fd, scalebuffer, sizeof(scalebuffer));
	float x0, y0, scaleX, scaleY;
	temp = (uint32_t)scalebuffer[0] | (uint32_t)scalebuffer[1] << 8 
		| (uint32_t)scalebuffer[2] << 16 | (uint32_t)scalebuffer[3] << 24;
	memcpy(&x0, &temp, sizeof(x0));
	temp = (uint32_t)scalebuffer[4] | (uint32_t)scalebuffer[5] << 8 
		| (uint32_t)scalebuffer[6] << 16 | (uint32_t)scalebuffer[7] << 24;
	memcpy(&y0, &temp, sizeof(y0));
	temp = (uint32_t)scalebuffer[8] | (uint32_t)scalebuffer[9] << 8 
		| (uint32_t)scalebuffer[10] << 16 | (uint32_t)scalebuffer[11] << 24;
	memcpy(&scaleX, &temp, sizeof(scaleX));
	temp = (uint32_t)scalebuffer[12] | (uint32_t)scalebuffer[13] << 8 
		| (uint32_t)scalebuffer[14] << 16 | (uint32_t)scalebuffer[15] << 24;
	memcpy(&scaleY, &temp, sizeof(scaleY));

	fat_close_file(fd);

	// Load the BMP image
	// TODO File selection based on lat/long
	selectLCD();
	loadBMP(fs, dd, MAPFILE, fd, &imgWidth, &imgHeight);

	while (1) {
		response = GPS::loadData(gpsdata);

		// Redraw, mainly for fast update
		switch (currentMode) {
			case MODE_MENU:
				if (redraw) {
					LCD::fillRect(0, 0, 16, 160, WHITE);
					// Draw the menu cursor
					LCD::fillRect(11, 26 + menuSelection * 10, 16, 31 + menuSelection * 10, RED);
					redraw = false;
				}
				break;
			case MODE_SETTINGS:
				if (redraw) {
					LCD::fillRect(15, 20, 22, 160, WHITE);
					// Draw the menu cursor
					LCD::fillRect(16, settingsSelection*25 + 26, 21, settingsSelection*25 +31, RED);
					redraw = false;
				}
				break;
			case MODE_MAPVIEW:
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
						if (gpsdata.status == GPS_NOFIX) 
							LCD::drawString("No Fix  ", 5, 5, RED);
						else if (gpsdata.status == GPS_GPSFIX)
							LCD::drawString("GPS Fix ", 5, 5, BLUE);
						else if (gpsdata.status == GPS_DIFF)
							LCD::drawString("Diff GPS", 5, 5, BLUE);
						LCD::drawString("# Sats:", 70, 5, BLACK);
						LCD::drawInt(gpsdata.numsats, 2, 100, 5, BLUE);
						LCD::drawString("Time:", 5, 15, BLACK);
						// Draw time in format
						uint8_t hour = gpsdata.time/10000;
						uint8_t min = (gpsdata.time-10000*hour)/100;
						uint8_t sec = (gpsdata.time-10000*hour-100*min);
						LCD::drawString("  :  :", 38, 15, BLACK);
						LCD::drawInt(hour, 2, 26, 15, BLUE);
						LCD::drawInt(min, 2, 44, 15, BLUE);
						LCD::drawInt(sec, 2, 62, 15, BLUE);
						LCD::drawString("Altitude:", 5, 25, BLACK);
						LCD::drawInt(gpsdata.altitude, 5, 65, 25, BLUE);
						if (UI_Mode == 1) {
							LCD::drawString("Lat:", 5, 35, BLACK);
							LCD::drawFloat(gpsdata.latitude, 38, 35, BLUE);
							LCD::drawString("Long:", 5, 45, BLACK);
							LCD::drawFloat(gpsdata.longitude, 38, 45, BLUE);
						}
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

				if (response == NMEA_GPGGA) {
					redraw = true;
					if (latitude == 0) {
						latitude = gpsdata.latitude;
						longitude = gpsdata.longitude;
					}
				}

				break;
			case MODE_SCROLLVIEW:
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
						LCD::setOrientation(0);
						LCD::drawString("Lat:", 5, 5, BLACK);
						LCD::drawFloat(latitude, 38, 5, BLUE);
						LCD::drawString("Long:", 5, 15, BLACK);
						LCD::drawFloat(longitude, 38, 15, BLUE);
					} else {
						// Draw the current lat and long 
						LCD::setOrientation(0);
						LCD::drawString("Lat:", 5, 5, BLACK);
						LCD::drawFloat(latitude, 38, 5, BLUE);
						LCD::drawString("Long:", 5, 15, BLACK);
						LCD::drawFloat(longitude, 38, 15, BLUE);
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
	return 0;
}

void handleInputs() {
	// Handle button inputs
	if (!(PIND & (1 << MENUBTN))) {
		switch (currentMode) {
			case MODE_MENU:
				if (menuSelection == MODE_MAPVIEW) {
					currentMode = MODE_MAPVIEW;
					lastOffsetX = 0;
					lastOffsetY = 0;
					loadBMP(fs, dd, MAPFILE, fd, &imgWidth, &imgHeight);
				} else if (menuSelection == MODE_SCROLLVIEW) {
					currentMode = MODE_SCROLLVIEW;
					lastOffsetX = gpsdata.latitude;
					lastOffsetY = gpsdata.longitude;
					loadBMP(fs, dd, MAPFILE, fd, &imgWidth, &imgHeight);
				} else if (menuSelection == MODE_ABOUT) {
					currentMode = MODE_ABOUT;
					// Display Alectyron logo
					LCD::clearDisp();
					drawAlectryonLogo(44, 0);
					strcpy_P(strbuffer, ABOUT_0);
					LCD::drawString(strbuffer, 35, 45, BLACK);
					strcpy_P(strbuffer, ABOUT_1);
					LCD::drawString(strbuffer, 25, 55, BLACK);

					// Display firmware and hardware versions
					strcpy_P(strbuffer, ABOUT_2);
					LCD::drawString(strbuffer, 15, 70, BLACK);
					LCD::drawString(FIRMWARE_VER, 53, 80, BLACK);
					strcpy_P(strbuffer, ABOUT_3);
					LCD::drawString(strbuffer, 15, 95, BLACK);
					LCD::drawString(HARDWARE_VER, 53, 105, BLACK);
					strcpy_P(strbuffer, ABOUT_4);
					LCD::drawString(strbuffer, 30, 120, BLACK);
					strcpy_P(strbuffer, ABOUT_5);
					LCD::drawString(strbuffer, 35, 130, BLACK);
					strcpy_P(strbuffer, ABOUT_6);
					LCD::drawString(strbuffer, 30, 140, BLACK);
				} else if (menuSelection == MODE_SDCARD) {
					// TODO: Some cool stuff with the files?
					currentMode = MODE_SDCARD;
					// Draw the files in the SDCARD
					LCD::clearDisp();
					LCD::drawString("SD Card: ", 10, 10, BLACK);
					fat_reset_dir(dd);
					fat_dir_entry_struct dir_entry;
					uint8_t drawRow = 0;
					while(fat_read_dir(dd, &dir_entry)) {
						selectLCD();
						LCD::drawString(dir_entry.long_name, 25, 25 + drawRow*10, BLACK);
						unselectLCD();
						++drawRow;
					}
					selectLCD();
				} else if (menuSelection == MODE_SETTINGS) {
					// Draw setting options and handle saving/loading into EEPROM
					// Setings: LCD brightness, default Lat/Long
					currentMode = MODE_SETTINGS;
					settingsSelection = SETTINGS_BRIGHTNESS;
					selectLCD();
					LCD::clearDisp();
					LCD::drawString("Settings", 10, 10, BLACK);
					
					for (uint8_t i = 0; i < settingsSize; ++i) {
						strcpy_P(strbuffer, (PGM_P)pgm_read_word(&(settingsItems[i])));
						LCD::drawString(strbuffer, 25, 25*(i+1), BLACK);
					}

					// Draw the values from the EEPROM
					LCD::drawInt(LCD_Brightness, 3, 50, 35, BLUE);
					LCD::drawInt(GPS_Rate, 5, 56, 60, BLUE);
					strcpy_P(strbuffer, (PGM_P)pgm_read_word(&(uiModeItems[UI_Mode])));
					LCD::drawString(strbuffer, 50, 85, BLUE);

					LCD::fillRect(16, settingsSelection*25 + 26, 21, settingsSelection*25 +31, LIME);
				} else if (menuSelection == MODE_SLEEP) {
					// Turn on interrupt for power on
					DDRD &= ~(1 << DDD2);
					PORTD |= (1 << PORTD2);
					EICRA |= (1 << ISC01);
					EIMSK |= (1 << INT0);	// Turns on INT0

					// Turn on the processor
					lowPowerMode();
					_delay_ms(1000);

					// Set the AVR to low power
					set_sleep_mode(SLEEP_MODE_PWR_DOWN);
					cli();
					sleep_enable();
					sei();
					sleep_cpu();
					sleep_disable();	// Where program continues
					normalMode();
					_delay_ms(1000);
					EICRA &= ~(1 << ISC01);
					EIMSK = 0;	// Turn off the INT0
				}
				redraw = true;
				break;
			case MODE_MAPVIEW:
				// Display a menu
				drawMenu();
				menuSelection = MODE_MAPVIEW;
				currentMode = MODE_MENU;
				redraw = true;
				fat_close_file(fd);	// Close the image
				break;
			case MODE_SCROLLVIEW:
				// Do an overflow counter, if menuBtn held for long
				if (!(PIND & (1 << UPBTN)) && !(PIND & (1 << DOWNBTN))) {
					// Display a menu
					drawMenu();
					menuSelection = MODE_SCROLLVIEW;
					currentMode = MODE_MENU;
					redraw = true;
					fat_close_file(fd);	// Close the image
					// Delay to prevent multiple menu hits
					_delay_ms(500);
				} else 
					step = -0.0010;
				break;
			case MODE_SDCARD:
				drawMenu();
				menuSelection = MODE_SDCARD;
				currentMode = MODE_MENU;
				redraw = true;
				break;
			case MODE_ABOUT:
				drawMenu();
				menuSelection = MODE_ABOUT;
				currentMode = MODE_MENU;
				redraw = true;
				break;
			case MODE_SETTINGS:
				if (settingsSelection == SETTINGS_BRIGHTNESS) {
					if (currentSettingSelection == SETTINGS_MODE_MENU) {
						currentSettingSelection = SETTINGS_MODE_BRIGHTNESS;
						currentEditPlace = 0;
						// Change the brightness
						LCD::drawIntEdit(LCD_Brightness, currentEditPlace, 50, 35, BLUE, LIME);
					} else if (currentSettingSelection == SETTINGS_MODE_BRIGHTNESS) {
						if (++currentEditPlace >= BRIGHTNESS_DIGITS) {
							currentSettingSelection = SETTINGS_MODE_MENU;
							LCD::drawInt(LCD_Brightness, 3, 50, 35, BLUE);
						} else {
							LCD::drawIntEdit(LCD_Brightness, currentEditPlace, 50, 35, BLUE, LIME);
						}
					} 
				} else if (settingsSelection == SETTINGS_GPSRATE) {
					if (currentSettingSelection == SETTINGS_MODE_MENU) {
						currentSettingSelection = SETTINGS_MODE_GPSRATE;
						LCD::drawInt(GPS_Rate, 5, 56, 60, GREEN);
					} else if (currentSettingSelection == SETTINGS_MODE_GPSRATE) {
						currentSettingSelection = SETTINGS_MODE_MENU;
						LCD::drawInt(GPS_Rate, 5, 56, 60, BLUE);
					} 
				} else if (settingsSelection == SETTINGS_UI) {
					if (currentSettingSelection == SETTINGS_MODE_MENU) {
						currentSettingSelection = SETTINGS_MODE_UI;
						strcpy_P(strbuffer, (PGM_P)pgm_read_word(&(uiModeItems[UI_Mode])));
						LCD::drawString(strbuffer, 50, 85, GREEN);
					} else if (currentSettingSelection == SETTINGS_MODE_UI) {
						currentSettingSelection = SETTINGS_MODE_MENU;
						strcpy_P(strbuffer, (PGM_P)pgm_read_word(&(uiModeItems[UI_Mode])));
						LCD::drawString(strbuffer, 50, 85, BLUE);
					} 
				} else if (settingsSelection == SETTINGS_SAVE) {
					// Save the settings to EEPROM
					eeprom_write_byte(&EEPROM_LCD_Brightness, LCD_Brightness);
					eeprom_write_word(&EEPROM_GPS_Rate, GPS_Rate);
					eeprom_write_byte(&EEPROM_UI_Mode, UI_Mode);

					// Update the GPS Update rate
					GPS::setUpdateRate(GPS_Rate);

					// Return back to the main menu
					drawMenu();
					menuSelection = MODE_MAPVIEW;
					currentMode = MODE_MENU;
					redraw = true;
				} else if (settingsSelection == SETTINGS_EXIT) {
					// Return back to the main menu
					drawMenu();
					menuSelection = MODE_SETTINGS;
					currentMode = MODE_MENU;
					redraw = true;
				}
				break;
			default:
				break;
		}
		overflowCounter2 = 0;
	} else {
		switch (currentMode) {
			case MODE_MENU:
				break;
			case MODE_MAPVIEW:
				break;
			case MODE_SCROLLVIEW:
				step = 0.0010;
				break;
			default:
				break;
		}
	}
	if (!(PIND & (1 << UPBTN))) {
		switch (currentMode) {
			case MODE_MENU:
				if (menuSelection > 0) --menuSelection;
				else menuSelection = menuSize - 1;
				redraw = true;
				break;
			case MODE_SETTINGS:
				if (currentSettingSelection == SETTINGS_MODE_MENU) {
					if (settingsSelection > 0) --settingsSelection;
					else settingsSelection = settingsSize - 1;
					redraw = true;
				} else if (currentSettingSelection == SETTINGS_MODE_BRIGHTNESS) {
					if (currentEditPlace == 0 && LCD_Brightness <= 155) LCD_Brightness += 100;
					else if (currentEditPlace == 1 && LCD_Brightness <= 245) LCD_Brightness +=10;
					else if (currentEditPlace == 2 && LCD_Brightness <= 254) LCD_Brightness +=1;
					LCD::drawIntEdit(LCD_Brightness, currentEditPlace, 50, 35, BLUE, LIME);
					OCR0A = LCD_Brightness;
				} else if (currentSettingSelection == SETTINGS_MODE_GPSRATE) {
					if (GPS_Rate < 10000) GPS_Rate += 1000;
					LCD::drawInt(GPS_Rate, 5, 56, 60, GREEN);
				} else if (currentSettingSelection == SETTINGS_MODE_UI) {
					if (UI_Mode < NUM_UI_MODES - 1) ++UI_Mode;
					strcpy_P(strbuffer, (PGM_P)pgm_read_word(&(uiModeItems[UI_Mode])));
					LCD::drawString(strbuffer, 50, 85, GREEN);
				}
				break;
			case MODE_MAPVIEW:
				break;
			case MODE_SCROLLVIEW:
				latitude += step;
				redraw = true;
				break;
			default:
				break;
		}
		PORTC |= (1 << STATUS1);
		overflowCounter2 = 0;
	} else {
		switch (currentMode) {
			case MODE_MENU:
				break;
			case MODE_MAPVIEW:
				break;
			case MODE_SCROLLVIEW:
				break;
			default:
				break;
		}
		PORTC &= ~(1 << STATUS1);
	}
	if (!(PIND & (1 << DOWNBTN))) {
		switch (currentMode) {
			case MODE_MENU:
				if (menuSelection < menuSize - 1) ++menuSelection;
				else menuSelection = 0;
				redraw = true;
				break;
			case MODE_SETTINGS:
				if (currentSettingSelection == SETTINGS_MODE_MENU) {
					if (settingsSelection < settingsSize - 1) ++settingsSelection;
					else settingsSelection = 0;
					redraw = true;
				} else if (currentSettingSelection == SETTINGS_MODE_BRIGHTNESS) {
					if (currentEditPlace == 0 && LCD_Brightness >= 100) LCD_Brightness -= 100;
					else if (currentEditPlace == 1 && LCD_Brightness >=10) LCD_Brightness -=10;
					else if (currentEditPlace == 2 && LCD_Brightness >=1) LCD_Brightness -=1;
					LCD::drawIntEdit(LCD_Brightness, currentEditPlace, 50, 35, BLUE, LIME);
					OCR0A = LCD_Brightness;
				} else if (currentSettingSelection == SETTINGS_MODE_GPSRATE) {
					if (GPS_Rate > 1000) GPS_Rate -= 1000;
					LCD::drawInt(GPS_Rate, 5, 56, 60, GREEN);
				} else if (currentSettingSelection == SETTINGS_MODE_UI) {
					if (UI_Mode > 0) --UI_Mode;
					strcpy_P(strbuffer, (PGM_P)pgm_read_word(&(uiModeItems[UI_Mode])));
					LCD::drawString(strbuffer, 50, 85, GREEN);
				}
				break;
			case MODE_MAPVIEW:
				break;
			case MODE_SCROLLVIEW:
				longitude += step;
				redraw = true;
				break;
			default:
				break;
		}
		overflowCounter2 = 0;
		PORTC |= (1 << STATUS2);
	} else {
		switch (currentMode) {
			case MODE_MENU:
				break;
			case MODE_MAPVIEW:
				break;
			case MODE_SCROLLVIEW:
				break;
			default:
				break;
		}
		PORTC &= ~(1 << STATUS2);
	}
}

void drawMenu()
{
	LCD::setOrientation(0);
	LCD::clearDisp();
	LCD::drawString("Menu", 25, 10, BLACK);
	for (uint8_t i = 0; i < menuSize; ++i) {
		strcpy_P(strbuffer, (PGM_P)pgm_read_word(&(menuItems[i])));
		LCD::drawString(strbuffer, 25, 25 + i * 10, BLACK);
	}
	drawAlectryonLogo(44, 120);
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

void drawAlectryonLogo(const uint8_t x, const uint8_t y)
{
	// Alectryon Logo drawn below the main menu
	// Width=40, Height=40
	LCD::fillRect(x, y, x+40, y+40, WHITE);
	LCD::fillRect(x+10, y+7, x+11, y+13, RED);
	LCD::fillRect(x+11, y+5, x+13, y+16, RED);
	LCD::fillRect(x+13, y+6, x+14, y+16, RED);
	LCD::fillRect(x+14, y+3, x+16, y+17, RED);
	LCD::fillRect(x+16, y+4, x+17, y+17, RED);
	LCD::fillRect(x+17, y+6, x+18, y+17, RED);
	LCD::fillRect(x+18, y+10, x+28, y+39, RED);
	LCD::fillRect(x+19, y+8, x+23, y+9, RED);
	LCD::fillRect(x+20, y+7, x+22, y+7, RED);
	LCD::fillRect(x+21, y+6, x+22, y+6, RED);
	LCD::fillRect(x+25, y+7, x+27, y+9, RED);
	LCD::fillRect(x+26, y+4, x+27, y+7, RED);
	LCD::fillRect(x+27, y+2, x+28, y+4, RED);
	LCD::fillRect(x+28, y+9, x+29, y+37, RED);
	LCD::fillRect(x+29, y+7, x+30, y+9, RED);
	LCD::fillRect(x+30, y+4, x+31, y+6, RED);
	LCD::fillRect(x+30, y+10, x+32, y+33, RED);
	LCD::fillRect(x+32, y+8, x+33, y+10, RED);
	LCD::fillRect(x+33, y+12, x+34, y+24, RED);
	LCD::fillRect(x+34, y+12, x+35, y+19, RED);
	LCD::fillRect(x+36, y+16, x+36, y+17, RED);
	LCD::fillRect(x+35, y+22, x+35, y+25, RED);
	LCD::fillRect(x+35, y+22, x+35, y+25, RED);
	LCD::fillRect(x+36, y+24, x+36, y+25, RED);
	LCD::fillRect(x+33, y+26, x+33, y+31, RED);
	LCD::fillRect(x+30, y+34, x+30, y+37, RED);
	LCD::fillRect(x+31, y+34, x+31, y+35, RED);
	LCD::fillRect(x+0, y+36, x+17, y+39, RED);
	LCD::fillRect(x+1, y+34, x+17, y+35, RED);
	LCD::fillRect(x+2, y+32, x+17, y+33, RED);
	LCD::fillRect(x+2, y+32, x+17, y+33, RED);
	LCD::fillRect(x+3, y+31, x+17, y+31, RED);
	LCD::fillRect(x+4, y+30, x+17, y+30, RED);
	LCD::fillRect(x+4, y+28, x+17, y+29, RED);
	LCD::fillRect(x+6, y+27, x+17, y+27, RED);
	LCD::fillRect(x+7, y+26, x+17, y+26, RED);
	LCD::fillRect(x+8, y+25, x+17, y+25, RED);
	LCD::fillRect(x+9, y+24, x+17, y+24, RED);
	LCD::fillRect(x+10, y+23, x+17, y+23, RED);
	LCD::fillRect(x+11, y+22, x+17, y+22, RED);
	LCD::fillRect(x+12, y+21, x+17, y+21, RED);
	LCD::fillRect(x+14, y+20, x+17, y+20, RED);
	LCD::fillRect(x+15, y+19, x+17, y+19, RED);
	LCD::fillRect(x+17, y+18, x+17, y+18, RED);
}

void readMasterFile(fat_fs_struct *fs, fat_dir_struct* dd, fat_file_struct *fd)
{
	// Read the master.list file containing filenames with lat/long scale and (0,0)
	// This file holds the geo-referencing data
	// Format: 
	// 4 bytes = lat of x0
	// 4 bytes = long of y0
	// 4 bytes = scale X (lat per pixel)
	// 4 bytes = scale Y (long per pixel)
	fd = open_file_in_dir(fs, dd, "master.list");
	if (!fd) {
		// Error opening file
		selectLCD();
		LCD::drawString("Error", 50, 70, BLACK);
		unselectLCD();
		return;
	}
	uint8_t buffer[16];
	fat_read_file(fd, buffer, sizeof(buffer));
	uint32_t temp;
	float m_x0, m_y0, m_scaleX, m_scaleY;
	temp = (uint32_t)buffer[0] | (uint32_t)buffer[1] << 8 
		| (uint32_t)buffer[2] << 16 | (uint32_t)buffer[3] << 24;
	memcpy(&m_x0, &temp, sizeof(m_x0));
	temp = (uint32_t)buffer[4] | (uint32_t)buffer[5] << 8 
		| (uint32_t)buffer[6] << 16 | (uint32_t)buffer[7] << 24;
	memcpy(&m_y0, &temp, sizeof(m_y0));
	temp = (uint32_t)buffer[8] | (uint32_t)buffer[9] << 8 
		| (uint32_t)buffer[10] << 16 | (uint32_t)buffer[11] << 24;
	memcpy(&m_scaleX, &temp, sizeof(m_scaleX));
	temp = (uint32_t)buffer[12] | (uint32_t)buffer[13] << 8 
		| (uint32_t)buffer[14] << 16 | (uint32_t)buffer[15] << 24;
	memcpy(&m_scaleY, &temp, sizeof(m_scaleY));

	fat_close_file(fd);
}

void loadBMP(fat_fs_struct *fs, fat_dir_struct* dd, const char* file, fat_file_struct *fd, 
		int16_t *width, int16_t *height)
{
	// Read the file
	fd = open_file_in_dir(fs, dd, file);
	if (!fd) {
		// Error opening file
		selectLCD();
		LCD::drawString("Error Opening SD Card", 10, 100, BLACK);
		unselectLCD();
		_delay_ms(1000);
	}

	// Read the bmp header
	intptr_t ret = fat_read_file(fd, buffer, HEADERSIZE);
	if (ret == -1) {
		selectLCD();
		LCD::drawString("Error Opening File", 10, 100, BLACK);
		unselectLCD();
		_delay_ms(1000);
	}
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
	//UART::initUART(9600, true);
	selectLCD();
	// Turn on LCD and PWM backlight
	LCD::writeCmd(SLPOUT);
	TCCR0A |= (1 << WGM00) | (1 << WGM01) | (1 << COM0A1);
	TCCR0B |= (1 << CS01);

	GPS::wakeup();
}

ISR (INT0_vect)
{

}

ISR (TIMER2_OVF_vect)
{
	++overflowCounter2;
}
