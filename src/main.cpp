#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <SPI.h>
#include <SD.h>

// Set pins for I2C comm on ESP32
#define I2C_SDA 21
#define I2C_SCL 22

// BMP I2C address found in data sheet
#define BMP280_I2C_ADDR 0x76

// Chip select pin for SD card
#define chipSelect 5

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

        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_1000);

    // Initialize SPI
    SPI.begin();
    
    if (!SD.begin(chipSelect))
    {
        Serial.println("SD card initialization failed!");
        while (1);
    }
    Serial.println("SD card initialized succesfully.");
}

void loop()
{
    float temperature = bmp.readTemperature();
    float pressure = bmp.readPressure() / 100; // Converts Pa to HPa
    float altitude = bmp.readAltitude();
 
    Serial.print("Temperature = ");
    Serial.print(temperature);
    Serial.println(" *C");
 
    Serial.print("Pressure = ");
    Serial.print(pressure);
    Serial.println(" hPa");
 
    Serial.print("Approx. Altitude = ");
    Serial.print(altitude);
    Serial.println(" m");

 // SD card module will use SPI comms
    File dataFile = SD.open("/sensor_data.txt", FILE_WRITE);

    if (dataFile)
    {
        dataFile.print("Temperature = ");
        dataFile.print(temperature);
        dataFile.println(" *C");
 
        dataFile.print("Pressure = ");
        dataFile.print(pressure);
        dataFile.println(" hPa");
 
        dataFile.print("Approx. Altitude = ");
        dataFile.print(altitude);
        dataFile.println(" m");
    }
    else
    {
        Serial.println("error opening sensor_data.txt");
    }

 delay(1000);
}