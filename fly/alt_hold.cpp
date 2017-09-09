# include "alt_hold.h"
#include <Wire.h>
#include <Adafruit_BMP085.h>


Adafruit_BMP085 bmp;
double PA;


bool altInit()
{
	if (!bmp.begin()) {
	Serial.println("Could not find a valid BMP085 sensor, check wiring!");
	while (1) {}
    }
    PA = bmp.readPressure();
    return true;
}
  
double get_altitude(long num) {
	double alt=bmp.readAltitude(num);
	return alt;
}
