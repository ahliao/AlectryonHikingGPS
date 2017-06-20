#ifndef _GPSDATA_H
#define _GPSDATA_H

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include <string.h>
#include "../comms/UART.hpp"

#define GPS_NOFIX 0
#define GPS_GPSFIX 1
#define GPS_DIFF 2

#define NMEA_NONE 0
#define NMEA_GPGGA 1

const char CMD_OUTPUT[] PROGMEM= "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*2C\r\n";
const char CMD_STANDBY[] PROGMEM = "$PMTK161,0*28\r\n";

const char CMD_RATE1[] PROGMEM = "$PMTK200,1000*1F\r\n";
const char CMD_RATE2[] PROGMEM = "$PMTK200,2000*1F\r\n";
const char CMD_RATE3[] PROGMEM = "$PMTK200,3000*1F\r\n";
const char CMD_RATE4[] PROGMEM = "$PMTK200,4000*1F\r\n";
const char CMD_RATE5[] PROGMEM = "$PMTK200,5000*1F\r\n";
const char CMD_RATE6[] PROGMEM = "$PMTK200,6000*1F\r\n";
const char CMD_RATE7[] PROGMEM = "$PMTK200,7000*1F\r\n";
const char CMD_RATE8[] PROGMEM = "$PMTK200,8000*1F\r\n";
const char CMD_RATE9[] PROGMEM = "$PMTK200,9000*1F\r\n";
const char CMD_RATE10[] PROGMEM = "$PMTK200,10000*1F\r\n";
PGM_P const CMD_RATES[] PROGMEM = 
{
	CMD_RATE1,
	CMD_RATE2,
	CMD_RATE3,
	CMD_RATE4,
	CMD_RATE5,
	CMD_RATE6,
	CMD_RATE7,
	CMD_RATE8,
	CMD_RATE9,
	CMD_RATE10
};
static char gpscmdstr[56];

/**
	GPS support functions for NMEA
	Requires UART to be init
*/
namespace GPS
{
	// Data structure to hold the important GPS data
	struct GPSData {
		uint8_t status;	// 0=no fix, 1=GPS fix, 2=Diff
		uint8_t numsats; // Number of satellites
		uint16_t time;	// hhmmss.ss (without the decimal)
		uint16_t altitude; // Altitude above sea-level
		float HDOP;	// Horizontal dilution of precision
		double longitude; // llll.ll (without the decimal, sign represents N or S)
		double latitude; // yyyy.yy 
	};

	// Load the GPS data from UART
	uint8_t loadData(GPSData &gpsdata);

	// Set the output statement frequencies
	void setOutputs();

	// Set the update rate (depends on baudrate)
	void setUpdateRate(int16_t ms);

	// Set GPS into standby mode for power saving
	void enterStandby();

	// Wake up by sending any byte
	void wakeup();
}

#endif
