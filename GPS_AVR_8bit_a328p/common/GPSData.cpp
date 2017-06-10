#include "GPSData.hpp"

namespace GPS
{
	static char GPSBuffer[100];
	static uint8_t readIndex = 0;
	static char *token;
	static char *deg;

	NMEA loadData(GPSData &gpsdata) {
		NMEA response = NONE;

		if (UART::UART_newData) {
			GPSBuffer[readIndex] = UART::UART_data;
			UART::UART_newData = false;
			if (GPSBuffer[readIndex] == '\n') {
				GPSBuffer[readIndex+1] = '\0';

				// Get the GPS NMEA command
				token = strtok(GPSBuffer, ",");

				// If it is Global Positioning System Fix Data
				if (!strcmp(token, "$GPGGA")) {
					// UTC of position (time)
					token = strtok(NULL, ",");
					gpsdata.time = atoi(token);

					// Latitude of position
					token = strtok(NULL, ",");
					gpsdata.latStr = token;
					//token = strcat(token, strtok(NULL, ","));
					// First take the integer value divide 100
					gpsdata.latitude = atoi(token)/100;
					deg = strrchr(token, '.');
					deg -= 2;
					gpsdata.latitude += atof(deg)/60;

					// North or South (South is negative)
					token = strtok(NULL, ",");
					if (!strcmp(token, "S"))
						gpsdata.latitude *= -1;

					// Longitude of position
					token = strtok(NULL, ",");
					gpsdata.longStr = token;
					//token = strcat(token, strtok(NULL, ","));
					gpsdata.longitude = atoi(token)/100;
					deg = strrchr(token, '.');
					deg -= 2;
					gpsdata.longitude += atof(deg)/60;

					// East or West (West is negative)
					token = strtok(NULL, ",");
					if (!strcmp(token, "W"))
						gpsdata.longitude *= -1;

					// GPS quality/status
					token = strtok(NULL, ",");
					gpsdata.status = atoi(token);

					// # of Satellites
					token = strtok(NULL, ",");
					gpsdata.numsats = atoi(token);

					// Horizontal dilution of precision
					token = strtok(NULL, ",");

					// Antenna altitude above mean sea level
					token = strtok(NULL, ",");
					gpsdata.altitude = atoi(token);

					// Altitude units (meters)
					token = strtok(NULL, ",");
				
					response = GPGGA;
				}
				for (uint8_t a = 0; a < strlen(GPSBuffer); ++a) {
					GPSBuffer[a] = 0;
				}
				readIndex = 0;
			} else {
				readIndex++;
			}
		}
		return response;
	}

	// Set the output statement frequencies
	// Sets so only NMEA GGA statements are 1 per fix
	void setOutputs()
	{
		char str[] = "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*2C\r\n";
		for (uint8_t i = 0; i < strlen(str); ++i) {
			UART::writeByte(str[i]);
		}
	}

	// Set the update rate (depends on baudrate)
	void setUpdateRate(int16_t ms)
	{
		const char *str;
		// Switch statement easier
		switch(ms) {
			case 1000:
				str = "$PMTK200,1000*1F\r\n";
				break;
			case 2000:
				str = "$PMTK200,2000*1F\r\n";
				break;
			case 3000:
				str = "$PMTK200,3000*1F\r\n";
				break;
			case 4000:
				str = "$PMTK200,4000*1F\r\n";
				break;
			case 5000:
				str = "$PMTK200,5000*1F\r\n";
				break;
			case 6000:
				str = "$PMTK200,6000*1F\r\n";
				break;
			case 7000:
				str = "$PMTK200,7000*1F\r\n";
				break;
			case 8000:
				str = "$PMTK200,8000*1F\r\n";
				break;
			case 9000:
				str = "$PMTK200,9000*1F\r\n";
				break;
			case 10000:
				str = "$PMTK200,10000*1F\r\n";
				break;
			default:
				str = "$PMTK200,1000*1F\r\n";
				break;
		}
		for (uint8_t i = 0; i < strlen(str); ++i) {
			UART::writeByte(str[i]);
		}
	}

	// Set GPS into standby mode for power saving
	void enterStandby()
	{
		char str[] = "$PMTK161,0*28\r\n";
		for (uint8_t i = 0; i < strlen(str); ++i) {
			UART::writeByte(str[i]);
		}
	}

	// Wake up by sending any byte
	void wakeup()
	{
		UART::writeByte('a');
	}
}
