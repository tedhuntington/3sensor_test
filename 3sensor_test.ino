#include "Adafruit_Si7021.h"
#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_SHT31.h"
//bme280
//#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>


//Si7021
Adafruit_Si7021 si7021 = Adafruit_Si7021();
//SHT31
Adafruit_SHT31 sht31 = Adafruit_SHT31();
//BME280
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

unsigned long delayTime;


void setup() {
  Serial.begin(115200);

  // wait for serial port to open
  while (!Serial) {
    delay(10);
  }

//===============
  Serial.println("Si7021 test!");
  
  if (!si7021.begin()) {
    Serial.println("Did not find Si7021 sensor!");
    while (true)
      ;
  }

  Serial.print("Found model ");
  switch(si7021.getModel()) {
    case SI_Engineering_Samples:
      Serial.print("SI engineering samples"); break;
    case SI_7013:
      Serial.print("Si7013"); break;
    case SI_7020:
      Serial.print("Si7020"); break;
    case SI_7021:
      Serial.print("Si7021"); break;
    case SI_UNKNOWN:
    default:
      Serial.print("Unknown");
  }
  Serial.print(" Rev(");
  Serial.print(si7021.getRevision());
  Serial.print(")");
  Serial.print(" Serial #"); Serial.print(si7021.sernum_a, HEX); Serial.println(si7021.sernum_b, HEX);
//=========================

  Serial.println("SHT31 test");
  if (!sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
    Serial.println("Couldn't find SHT31");
    while (1) delay(1);
  }

//===================
    unsigned status;
    
    // default settings
    status = bme.begin();  
    // You can also pass in a Wire library object like &Wire2
    // status = bme.begin(0x76, &Wire2)
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1) delay(10);
    }


  
}

void loop() {

  Serial.println("Si7021");
  Serial.print("Temperature:\t");
  Serial.println(si7021.readTemperature(), 2);  
  Serial.print("Humidity:\t");
  Serial.println(si7021.readHumidity(), 2);

//================
  Serial.println("SHT31");
  float t = sht31.readTemperature();
  float h = sht31.readHumidity();

  if (! isnan(t)) {  // check if 'is not a number'
    Serial.print("Temperature:\t"); 
    Serial.println(t);
  } else { 
    Serial.println("Failed to read temperature");
  }
  
  if (! isnan(h)) {  // check if 'is not a number'
    Serial.print("Humidity:\t"); 
    Serial.println(h);
  } else { 
    Serial.println("Failed to read humidity");
  }
//=====================
    Serial.println("BME280");

    Serial.print("Temperature:\t");
    Serial.println(bme.readTemperature());
    
    //Serial.print("Pressure:\t");
    //Serial.println(bme.readPressure() / 100.0F);
    //Serial.println(" hPa");

    //Serial.print("Approx. Altitude = ");
    //Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    //Serial.println(" m");

    Serial.print("Humidity:\t");
    Serial.println(bme.readHumidity());
    //Serial.println(" %");


//==========================
  
  Serial.println();

  
  
  delay(1000);
}
