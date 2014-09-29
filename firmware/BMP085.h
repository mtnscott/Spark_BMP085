/***************************** spark_BMP085.h ************************************/
/*
 * spark_BMP085.h
 *
 *  Created on: Jan 22, 2014
 *      Author: scottpiette
 */

#ifndef SPARK_BMP085_H
#define SPARK_BMP085_H

#include <application.h>

/******************************** spark_bmp.h ************************************/

#define BMP085_SAMPLETIME		 1500  // Set a sample refresh time to 1500ms

#define BMP085_I2CADDR			 0x77

#define BMP085_ULTRALOWPOWER		0
#define BMP085_STANDARD				1
#define BMP085_HIGHRES				2
#define BMP085_ULTRAHIGHRES			3
#define BMP085_CAL_AC1           0xAA  // R   Calibration data (16 bits)
#define BMP085_CAL_AC2           0xAC  // R   Calibration data (16 bits)
#define BMP085_CAL_AC3           0xAE  // R   Calibration data (16 bits)
#define BMP085_CAL_AC4           0xB0  // R   Calibration data (16 bits)
#define BMP085_CAL_AC5           0xB2  // R   Calibration data (16 bits)
#define BMP085_CAL_AC6           0xB4  // R   Calibration data (16 bits)
#define BMP085_CAL_B1            0xB6  // R   Calibration data (16 bits)
#define BMP085_CAL_B2            0xB8  // R   Calibration data (16 bits)
#define BMP085_CAL_MB            0xBA  // R   Calibration data (16 bits)
#define BMP085_CAL_MC            0xBC  // R   Calibration data (16 bits)
#define BMP085_CAL_MD            0xBE  // R   Calibration data (16 bits)

#define BMP085_CONTROL			 0xF4
#define BMP085_TEMPDATA			 0xF6
#define BMP085_PRESSUREDATA		 0xF6
#define BMP085_READTEMPCMD		 0x2E
#define BMP085_READPRESSURECMD	 0x34

class spark_BMP {
	public:
		spark_BMP();
		bool init(uint8_t mode = BMP085_ULTRAHIGHRES);  // by default go highres
		float Temperature(void);
		int32_t Pressure(void);
		float Altitude(float sealevelPressure = 101325); // std atmosphere

	private:
		bool refresh(uint32_t currTime);								/* gets values from device */
		uint16_t readRawTemperature(void);			/* internal read temperature */
		uint32_t readRawPressure(void);					/* internal read pressure */
		uint8_t read8(uint8_t addr);							/* wire read 1 byte */
		uint16_t read16(uint8_t addr);						/* wire read 2 bytes */
		void write8(uint8_t addr, uint8_t data);		/* wire write 1 byte */
		int32_t _pressure;											/* last pressure read */
		float _temperature;										/* last temperature read */
		uint32_t _sampletime;									/* sample time */
		uint8_t _oversampling;									/* BMP sample quality setting */
		int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;	/* calibration numbers */
		uint16_t ac4, ac5, ac6;									/* calibration numbers */
};
#endif

