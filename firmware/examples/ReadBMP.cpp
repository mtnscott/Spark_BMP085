// This #include statement was automatically added by the Spark IDE.
#include "BMP085/BMP085.h"

/*
 *  BMP 085 Example test applicaiton
 *  Scott Piette
 *  Sept 30, 2014
 */

/*
 Wiring
 ------
 BMP085 Vcc to 3.3V
 BMP085 GND to GND
 BMP085 SCL to D1
 BMP085 SDA to D0
 */

spark_BMP bmp;

// Initialize BMP085

void InitializeBMP085(){
	if (!bmp.init()) {
		Serial.println("Could not find a valid BMP085 sensor, check wiring!");
		while (1) {}
	}
}

// Publish Pressure, Altitude
void PublishBMP085Info(){
    Serial.print("Temperature = ");
    Serial.print(bmp.Temperature());
    Serial.println(" *C");
    
    Serial.print("Pressure = ");
    Serial.print(bmp.Pressure());
    Serial.println(" Pa");
    
    // Calculate altitude assuming 'standard' barometric
    // pressure of 1013.25 millibar = 101325 Pascal
    Serial.print("Altitude = ");
    Serial.print(bmp.Altitude());
    Serial.println(" meters");
    
    // you can get a more precise measurement of altitude
    // if you know the current sea level pressure which will
    // vary with weather and such. If it is 1015 millibars
    // that is equal to 101500 Pascals.
    Serial.print("Real altitude = ");
    Serial.print(bmp.Altitude(101500));
    Serial.println(" meters");
    
    char szEventInfo[64];
    
    sprintf(szEventInfo, "Temperature=%.2f °C, Pressure=%.2f hPa", bmp.Temperature(), bmp.Pressure()/100.0);
    
    Spark.publish("bmpo85info", szEventInfo);
}

// Initialize applicaiton
void InitializeApplication(){
    Serial.begin(9600);
    while (!Serial.available())
        ;       // Wait for serial input
	pinMode(D7, OUTPUT);
    
    Serial.println("Begin Debugging...");
}

// Blink LED and wait for some time
void BlinkLED(){
    digitalWrite(D7, HIGH);
    delay(500);
    digitalWrite(D7, LOW);
    delay(500);
}

void setup() {
    InitializeApplication();
    
    Serial.println("Init BMP");
    InitializeBMP085();
}

void loop() {
    // Publish events. Wait for 2 second between publishes
    PublishBMP085Info();
    
    BlinkLED();
	
	delay(2000);
}