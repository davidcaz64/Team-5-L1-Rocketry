#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>

// I2C Pins on ESP32
#define I2C_SDA 21
#define I2C_SCL 22

// BMP I2C address found in data sheet
#define BMP280_I2C_ADDR 0x76

Adafruit_BMP280 bmp;

void setup()
{
    Serial.begin(9600); // baud rate

    // Initialize I2C
    Wire.begin(I2C_SDA, I2C_SCL);

    if (!bmp.begin(BMP280_I2C_ADDR))
    {
        Serial.println("Could not find a valid BMP280 senor, check wiring!");
        while (1);
    }

}

void loop()
{
 float temperature = bmp.readTemperature();
 float pressure = bmp.readPressure();
 // float altitude = bmp.readAltitude;
 
 //Serial.print

 delay(2000);
}