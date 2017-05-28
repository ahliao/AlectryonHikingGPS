#include "GPSData.hpp"

namespace GPS
{
	static char GPSBuffer[100];
	static uint8_t readIndex = 0;
	static char *token;

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
					//token = strcat(token, strtok(NULL, ","));
					gpsdata.latitude = atof(token);

					// North or South (South is negative)
					token = strtok(NULL, ",");
					if (!strcmp(token, "S"))
						gpsdata.latitude *= -1;

					// Longitude of position
					token = strtok(NULL, ",");
					//token = strcat(token, strtok(NULL, ","));
					gpsdata.longitude = atof(token);

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
}