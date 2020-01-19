// Author: Alex Stubbles
// Description: This code echos sensor data from an accelerometer, thermometer, and altimeter to the computer
// TODO: print GPS as well
// Thermometer/Altimeter: BPM388
// GPS: Ultimate GPS Breakout v3
// Accelerometer: LSM9DS1 9-DOF
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include <SoftwareSerial.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_LSM9DS1.h>
// *******************GPS SETUP*******************
SoftwareSerial mySerial(9, 6); // (TX, RX)
Adafruit_GPS GPS(&mySerial);
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,1000*2F"
// send only RMC signal
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define PMTK_Q_RELEASE "$PMTK605*31"
// *******************TEMP/ALT SETUP*******************
// define pins for Adalogger 32u4
#define BMP_SCK A3
#define BMP_MISO A2
#define BMP_MOSI A1
#define BMP_CS A0
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BMP3XX bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
// *******************ACCEL SETUP*******************
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

void setup() {
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

void loop() {
  delay(250);
  lsm.read(); // read from IMU
  bmp.performReading(); // read from Altimeter/Thermometer
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);
  // Pull data from 
  Serial.print("Temperature: ");
  Serial.println(bmp.temperature);
  Serial.print("Pressure: ");
  Serial.println(bmp.pressure);
  Serial.println("Acceleration");
  Serial.println(a.acceleration.x);
  Serial.println(a.acceleration.y);
  Serial.println(a.acceleration.z);
  Serial.println("Gyro");
  Serial.println(a.gyro.x);
  Serial.println(a.gyro.y);
  Serial.println(a.gyro.z);
  Serial.println("Magnetic Field");
  Serial.println(m.magnetic.x);
  Serial.println(m.magnetic.y);
  Serial.println(m.magnetic.z);
  Serial.println();
}
