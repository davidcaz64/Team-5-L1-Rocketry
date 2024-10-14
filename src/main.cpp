#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <SPI.h>
#include <SD.h>

// Set pins for I2C comms on ESP32
#define I2C_SDA 21
#define I2C_SCL 22

// BMP I2C address found in data sheet
#define BMP280_I2C_ADDR 0x76

// Chip select pin for SD card
#define SD_CS 5

Adafruit_BMP280 bmp;

void setup()
{
    Serial.begin(9600); // baud rate

    // Initialize I2C
    Wire.begin(I2C_SDA, I2C_SCL);
    
    if (!SD.begin(SD_CS))
    {
        Serial.println("SD card initialization failed!");
        while (1);
    }
    Serial.println("SD card initialized succesfully.");

    if (!bmp.begin(BMP280_I2C_ADDR))
    {
        Serial.println("Could not find a valid BMP280 senor, check wiring!");
        while (1);
    }

    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_1000);
       
}

void loop()
{
 float temperature = bmp.readTemperature();
 float pressure = bmp.readPressure();
 float altitude = bmp.readAltitude();
 
 Serial.println("Temperature = ");
 Serial.print(temperature);
 Serial.print(" *C");
 
 Serial.println("Pressure = ");
 Serial.print(pressure);
 Serial.print(" hPa");
 
 Serial.println("Approx. Altitude = ");
 Serial.print(altitude);
 Serial.print(" m");

 // SD card module will use SPI comms
 File dataFile = SD.open("sensor_data.txt", FILE_WRITE);
 if (dataFile)
 {
    dataFile.println("Temperature = ");
    dataFile.print(temperature);
    dataFile.print(" *C");
 
    dataFile.println("Pressure = ");
    dataFile.print(pressure);
    dataFile.print(" hPa");
 
    dataFile.println("Approx. Altitude = ");
    dataFile.print(altitude);
    dataFile.print(" m");
 }
 
 delay(2000);
}