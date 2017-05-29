#ifndef _GPSDATA_H
#define _GPSDATA_H

#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include "../comms/UART.hpp"

/**
	GPS support functions for NMEA
	Requires UART to be init
*/
namespace GPS
{
	enum NMEA{
		NONE,
		GPGGA
	};

	// Data structure to hold the important GPS data
	struct GPSData {
		uint8_t status;	// 0=no fix, 1=GPS fix, 2=Dif, 3=GPS Dif
		uint8_t numsats; // Number of satellites
		uint16_t time;	// hhmmss.ss (without the decimal)
		uint16_t altitude; // Altitude above sea-level
		float HDOP;	// Horizontal dilution of precision
		float longitude; // llll.ll (without the decimal, sign represents N or S)
		float latitude; // yyyy.yy 
		char *longStr;
		char *latStr;
	};

	// Load the GPS data from UART
	NMEA loadData(GPSData &gpsdata);
}

#endif
