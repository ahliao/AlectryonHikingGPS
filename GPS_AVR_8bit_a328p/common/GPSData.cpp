#include "GPSData.hpp"

namespace GPS
{
	static char GPSBuffer[100];
	static uint8_t readIndex = 0;
	static char *token;
	static char *deg;

	uint8_t loadData(GPSData &gpsdata) {
		uint8_t response = NMEA_NONE;

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
					//gpsdata.latStr = token;
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
					//gpsdata.longStr = token;
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
				
					response = NMEA_GPGGA;
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
		strcpy_P(gpscmdstr, CMD_OUTPUT);
		for (uint8_t i = 0; i < strlen(gpscmdstr); ++i) {
			UART::writeByte(gpscmdstr[i]);
		}
	}

	// Set the update rate (depends on baudrate)
	void setUpdateRate(int16_t ms)
	{
		// Switch statement easier
		strcpy_P(gpscmdstr, (PGM_P)pgm_read_word(&(CMD_RATES[ms/1000])));
		for (uint8_t i = 0; i < strlen(gpscmdstr); ++i) {
			UART::writeByte(gpscmdstr[i]);
		}
	}

	// Set GPS into standby mode for power saving
	void enterStandby()
	{
		strcpy_P(gpscmdstr, CMD_STANDBY);
		for (uint8_t i = 0; i < strlen(gpscmdstr); ++i) {
			UART::writeByte(gpscmdstr[i]);
		}
	}

	// Wake up by sending any byte
	void wakeup()
	{
		UART::writeByte(' ');
	}
}
