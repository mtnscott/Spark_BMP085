/******************************** spark_BMP085.cpp ************************************/
/*
 * spark_BMP085.cpp
 *
 *  Created on: Jan 22, 2014
 *      Author: scottpiette
 *      this app includes all functions to date :
 *
 *      >  reading barometric pressure & temp from the BMP085
 *      >  uptime function to keep track of the current running time
 *      >  reading temperature and humidity from the DHT11
 *      >  flashing an LED for a heart beat
 *
 *      Need to add
 *      >  creating, maintaining cloud variables
 *      >  using the Ultrasonic sensor to measure distances
 *      >  reading the CO sensor
 *      >  reading the PIR sensor to determine if people & motion are present
 *      >  reading the VGA camera to capture location of specific objects (ie - car location in garage)
 */

#include <application.h>
#include <math.h>
#include "BMP085.h"

spark_BMP::spark_BMP() {
}

bool spark_BMP::init(uint8_t mode) {

	if (mode > BMP085_ULTRAHIGHRES)
		mode = BMP085_ULTRAHIGHRES;
	_oversampling = mode;

	/*
	 *  There are 22 bytes of calibration data
	 *  Lets read them into the variables
	 */
	Wire.begin();

	/* Not sure what this is about, can't find any reference for this first read in the docs */
	if (read8(0xD0) != 0x55)	return false;

	/* Read calibration data from the device */
	ac1 = read16(0xAA);
	ac2 = read16(0xAC);
	ac3 = read16(0xAE);
	ac4 = read16(0xB0);
	ac5 = read16(0xB2);
	ac6 = read16(0xB4);
	b1 = read16(0xB6);
	b2 = read16(0xB8);
	mb = read16(0xBA);
	mc = read16(0xBC);
	md = read16(0xBE);

	_sampletime = 0;

	return true;
}

float spark_BMP::Temperature(void) {
	uint32_t currTime;

	currTime = millis();
	if (_sampletime + BMP085_SAMPLETIME < currTime)
		refresh(currTime);

	return _temperature;
}

int32_t spark_BMP::Pressure(void) {
	uint32_t currTime;

	currTime = millis();
	if (_sampletime + BMP085_SAMPLETIME < currTime)
		refresh(currTime);

	return _pressure;
}

float spark_BMP::Altitude(float sealevelPressure) {
	float altitude;
	uint32_t currTime;

	currTime = millis();
	if (_sampletime + BMP085_SAMPLETIME < currTime)
		refresh(currTime);

	altitude = 44330 * (1.0 - pow(_pressure /sealevelPressure,0.1903));

	return altitude;
}

/********************************* private functions ****************************************/

uint16_t spark_BMP::readRawTemperature(void) {
	write8(BMP085_CONTROL, BMP085_READTEMPCMD);
	delay(5);
	return read16(BMP085_TEMPDATA);
}

uint32_t spark_BMP::readRawPressure(void) {
	uint32_t raw;

	write8(BMP085_CONTROL, BMP085_READPRESSURECMD + (_oversampling << 6));

	delay(2+(3<<_oversampling));

	raw = read16(BMP085_PRESSUREDATA);

	raw <<= 8;
	raw |= read8(BMP085_PRESSUREDATA+2);
	raw >>= (8 - _oversampling);

	return raw;
}

#if 0
bool spark_BMP::refresh(uint32_t currTime) {
	int32_t UT, UP, B3, B5, B6, X1, X2, X3, p;
	uint32_t B4, B7;
	float temp;

	_sampletime = currTime;		/* set the current sample time */
	UT = readRawTemperature();
	UP = readRawPressure();

	// do temperature calculations
	X1=(UT-(int32_t)(ac6))*((int32_t)(ac5))/pow(2,15);
	X2=((int32_t)mc*pow(2,11))/(X1+(int32_t)md);
	B5=X1 + X2;
	temp = (B5+8)/pow(2,4);
	temp /= 10;


	// do pressure calcs
	B6 = B5 - 4000;
	X1 = ((int32_t)b2 * ( (B6 * B6)>>12 )) >> 11;
	X2 = ((int32_t)ac2 * B6) >> 11;
	X3 = X1 + X2;
	B3 = ((((int32_t)ac1*4 + X3) << _oversampling) + 2) / 4;

	X1 = ((int32_t)ac3 * B6) >> 13;
	X2 = ((int32_t)b1 * ((B6 * B6) >> 12)) >> 16;
	X3 = ((X1 + X2) + 2) >> 2;
	B4 = ((uint32_t)ac4 * (uint32_t)(X3 + 32768)) >> 15;
	B7 = ((uint32_t)UP - B3) * (uint32_t)( 50000UL >> _oversampling );

	if (B7 < 0x80000000) {
		p = (B7 * 2) / B4;
	} else {
		p = (B7 / B4) * 2;
	}

	X1 = (p >> 8) * (p >> 8);
	X1 = (X1 * 3038) >> 16;
	X2 = (-7357 * p) >> 16;

	p = p + ((X1 + X2 + (int32_t)3791)>>4);

	_temperature = temp;
	_pressure = p;

	return true;
}
#endif
// Lets try and calculate a different way

bool spark_BMP::refresh(uint32_t currTime) {
	int32_t UT, UP, B3, B5, B6, X1, X2, X3, p;
	uint32_t B4, B7;
	float temp;

	_sampletime = currTime;		/* set the current sample time */
	UT = readRawTemperature();
	UP = readRawPressure();

	// do temperature calculations
	X1 = ((long) UT - ac6) * ac5 >> 15;
	X2 = ((long) mc << 11) / (X1 + md);
	B5 = X1 + X2;
	_temperature = (B5 + 8) >> 4;

	// do pressure calcs
	B6 = B5 - 4000;
	X1 = (b2 * (B6 * B6 >> 12 )) >> 11;
	X2 = ac2 * B6 >> 11;
	X3 = X1 + X2;
	B3 = (((int32_t) ac1 * 4 + X3) << _oversampling + 2) >> 2;

	X1 = ac3 * B6 >> 13;
	X2 = (b1 * (B6 * B6 >> 12)) >> 16;
	X3 = ((X1 + X2) + 2) >> 2;
	B4 = (ac4 * (uint32_t) (X3 + 32768)) >> 15;
	B7 = ((uint32_t)UP - B3) * ( 50000UL >> _oversampling );

	p = B7 < 0x80000000 ? (B7 * 2) / B4 : (B7 / B4) * 2;

	X1 = (p >> 8) * (p >> 8);
	X1 = (X1 * 3038) >> 16;
	X2 = (-7357 * p) >> 16;

	_pressure = p + ((X1 + X2 + 3791) >> 4);

	return true;
}

/************************* i2c wire read and write functions ******************************/

uint8_t spark_BMP::read8(uint8_t a) {
	uint8_t ret;

	Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device
	Wire.write(a); // sends register address to read from
	Wire.endTransmission(); // end transmission

	Wire.requestFrom(BMP085_I2CADDR, 1);// send data n-bytes read
	while (!Wire.available())
		;
	ret = Wire.read(); // receive DATA

	return ret;
}

uint16_t spark_BMP::read16(uint8_t a) {
	uint16_t ret;

	Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device
	Wire.write(a); // sends register address to read from
	Wire.endTransmission(); // end transmission

	Wire.requestFrom(BMP085_I2CADDR, 2);// send data n-bytes read
	while (Wire.available()< 2)
		;
	ret = Wire.read(); // receive DATA
	ret <<= 8;
	ret |= Wire.read(); // receive DATA

	return ret;
}

void spark_BMP::write8(uint8_t a, uint8_t d) {
	Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device
	Wire.write(a); // sends register address to read from
	Wire.write(d);  // write data
	Wire.endTransmission(); // end transmission

}
