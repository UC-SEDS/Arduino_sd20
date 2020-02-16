#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_LSM9DS1.h>
#include <avr/dtostrf.h>
#include <SD.h>
#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#define SERVER_ADDRESS 50
#define chipSelect 4
// what's the name of the hardware serial port?
#define GPSSerial Serial1

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);
// *******************TEMP/ALT SETUP*******************
#define BMP_SCK A0
#define BMP_MISO A1
#define BMP_MOSI A2
#define BMP_CS A3
Adafruit_BMP3XX bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
// *******************ACCEL SETUP*******************
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
#define LED 13

// for feather m0  
#define RFM95_CS 10
#define RFM95_RST 11
#define RFM95_INT 6

RH_RF95 driver(RFM95_CS, RFM95_INT);;

RHReliableDatagram manager(driver, SERVER_ADDRESS);

char filename[15];
void setup()
{
  Serial.begin(9600);
  if (!manager.init()){
    Serial.println("init failed");
  }
  Serial.println("Setting freq");
  driver.setFrequency(915.0);
  driver.setTxPower(23, false);
//  driver.setModemConfig(RH_RF95::Bw31_25Cr48Sf512);
  manager.setTimeout(2000);
  if (!SD.begin(chipSelect)) {
    Serial.println("Card init. failed!");
  }
  strcpy(filename, "GPSLOG00.TXT");
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = '0' + i/10;
    filename[7] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }
  bmp.begin();
  delay(100);
  lsm.begin();
  delay(100);
  GPS.begin(9600);
  delay(100);
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  delay(100);
  GPS.sendCommand("$PMTK622,1*29");
}

const char tag[8] = "PEANUT,";
uint8_t buffr[RH_RF95_MAX_MESSAGE_LEN];
char radiopacket[200];
void loop()
{
  char buf[200];
  char gps[80];
  char acc[12];
  strcpy(buf, tag);
  char c = GPS.read();
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) return;
    strcpy(gps, GPS.lastNMEA());
    lsm.read();
    sensors_event_t a, m, g, temp;
    lsm.getEvent(&a, &m, &g, &temp);
    dtostrf(a.acceleration.x, 8, 5, acc);
    strcat(buf, acc);
    strcat(buf, ",");
    dtostrf(a.acceleration.y, 8, 5, acc);
    strcat(buf, acc);
    strcat(buf, ",");
    dtostrf(a.acceleration.z, 8, 5, acc);
    strcat(buf, acc);
    strcat(buf, ",");
    dtostrf(g.gyro.x, 8, 4, acc);
    strcat(buf, acc);
    strcat(buf, ",");
    dtostrf(g.gyro.y, 8, 4, acc);
    strcat(buf, acc);
    strcat(buf, ",");
    dtostrf(g.gyro.z, 8, 4, acc);
    strcat(buf, acc);
    strcat(buf, ",");
    dtostrf(m.magnetic.x, 7, 4, acc);
    strcat(buf, acc);
    strcat(buf, ",");
    dtostrf(m.magnetic.y, 7, 4, acc);
    strcat(buf, acc);
    strcat(buf, ",");
    dtostrf(m.magnetic.z, 7, 4, acc);
    strcat(buf, acc);
    strcat(buf, ",");
    bmp.performReading();
    dtostrf(bmp.temperature, 9, 4, acc);
    strcat(buf, acc);
    strcat(buf, ",");
    dtostrf(bmp.pressure, 9, 1, acc);
    strcat(buf, acc);
    strcat(buf, ",");
    strcat(buf, gps);
  }
  else{
    buf[0] = 0;
    gps[0] = 0;
    acc[0] = 0;
    return;
  }
  if (strlen(buf) > 200) return;
  Serial.print(buf);
//  File logfile = SD.open(filename, FILE_WRITE);
//  if (logfile) {
//    logfile.print(buf);
//    logfile.close();
//  }
  strcpy(radiopacket, buf);
  uint8_t len = sizeof(buffr);
  uint8_t from;
  if (manager.recvfromAck(buffr, &len, &from))
  {
    Serial.print("got reply from : 0x");
    Serial.print(from, HEX);
    Serial.print(" : RSSI ");
    Serial.print(driver.lastRssi());
    Serial.print(" : ");
    Serial.println((char*)buffr);

    if (!manager.sendtoWait((uint8_t*)radiopacket, sizeof(radiopacket), from))
      Serial.println("No ACK-ACK");
  }
}
