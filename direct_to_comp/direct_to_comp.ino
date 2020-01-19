//Author: Alex Stubbles
//Date Created: 1/12/2020
//Date Last Update: NA
//Description: This is a developement script for the electrical bays.
//The script will directly send telemetry data over the serial port 
//to the computer. This allows for testing of the GUI with all data 
//without running into meory issues on the Feather 32u4.

//TODO: resolder the temp/alt sensor and add it's data to the string

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include <SoftwareSerial.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_LSM9DS1.h>

SoftwareSerial mySerial(9, 6); // (TX, RX)
Adafruit_GPS GPS(&mySerial);
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,1000*2F"
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define PMTK_Q_RELEASE "$PMTK605*31"
// *******************TEMP/ALT SETUP*******************
#define BMP_SCK A3
#define BMP_MISO A2
#define BMP_MOSI A1
#define BMP_CS A0
Adafruit_BMP3XX bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
// *******************ACCEL SETUP*******************
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
#define LED 13

void setup()
{
  Serial.begin(115200);
  // Start sensors
  bmp.begin();
  lsm.begin();
  GPS.begin(9600);
  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  delay(2000);
  GPS.sendCommand(PMTK_Q_RELEASE);
  // Output RMC only at 10 hz
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
  // Setting IMU ranges
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}

//const char tag[7] = "PEANUT,";
const char tag[7] = "BUTTER,";

void loop()
{
  char buf[200]; // total output buffer to computer
  char gps[80]; // buffer for gps data
  char dat[12]; // buffer for individual data points
  strcpy(buf, tag); // add tag to the begining of data
  char c = GPS.read();
  // checks to see if new data is received then parses 
  // the data and saves it to the gps buffer.
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) return;
    strcpy(gps, GPS.lastNMEA());
  }
  else {
    buf[0] = 0;
    gps[0] = 0;
    dat[0] = 0;
    return;
  }
  // read IMU and set up 
  lsm.read();
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);
  dtostrf(a.acceleration.x, 8, 5, dat);
  strcat(buf, dat);
  strcat(buf, ",");
  dtostrf(a.acceleration.y, 8, 5, dat);
  strcat(buf, dat);
  strcat(buf, ",");
  dtostrf(a.acceleration.z, 8, 5, dat);
  strcat(buf, dat);
  strcat(buf, ",");
  dtostrf(g.gyro.x, 8, 4, dat);
  strcat(buf, dat);
  strcat(buf, ",");
  dtostrf(g.gyro.y, 8, 4, dat);
  strcat(buf, dat);
  strcat(buf, ",");
  dtostrf(g.gyro.z, 8, 4, dat);
  strcat(buf, dat);
  strcat(buf, ",");
  dtostrf(m.magnetic.x, 7, 4, dat);
  strcat(buf, dat);
  strcat(buf, ",");
  dtostrf(m.magnetic.y, 7, 4, dat);
  strcat(buf, dat);
  strcat(buf, ",");
  dtostrf(m.magnetic.z, 7, 4, dat);
  strcat(buf, dat);
  strcat(buf, ",");
  bmp.performReading();
  dtostrf(bmp.temperature, 9, 4, dat);
  strcat(buf, dat);
  strcat(buf, ",");
  dtostrf(bmp.pressure, 9, 1, dat);
  strcat(buf, dat);
  strcat(buf, ",");
  strcat(buf, gps);
  Serial.print(buf);
  Serial.flush();
}
