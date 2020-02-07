#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_LSM9DS1.h>
#include <avr/dtostrf.h>
#include <SD.h>
#include <RH_RF95.h>
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

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  true

#define RFM95_RST     11   // "A"
#define RFM95_CS      10   // "B"
#define RFM95_INT     6    // "D" 

#if defined(ESP8266)
  /* for ESP w/featherwing */ 
  #define RFM95_CS  2    // "E"
  #define RFM95_RST 16   // "D"
  #define RFM95_INT 15   // "B"

#elif defined(ESP32)  
  /* ESP32 feather w/wing */
  #define RFM95_RST     27   // "A"
  #define RFM95_CS      33   // "B"
  #define RFM95_INT     12   //  next to A

#elif defined(NRF52)  
  nRF52832 feather w/wing
  #define RFM95_RST     7   // "A"
  #define RFM95_CS      11   // "B"
  #define RFM95_INT     31   // "C"
  
#elif defined(TEENSYDUINO)
  /* Teensy 3.x w/wing */
  #define RFM95_RST     9   // "A"
  #define RFM95_CS      10   // "B"
  #define RFM95_INT     4    // "C"
#endif

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

char filename[15];
void setup()
{
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  
  Serial.begin(115200);
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
  File logfile = SD.open(filename, FILE_WRITE);
  delay(100);
  bmp.begin();
  delay(100);
  lsm.begin();
  delay(100);
  GPS.begin(9600);
  delay(100);
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  rf95.setTxPower(23, false);
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  delay(1000);
  GPS.sendCommand("$PMTK622,1*29");
}
const char tag[8] = "PEANUT,";
void loop()                     // run over and over again
{
  char buf[200];
  char gps[80];
  char acc[12];
  strcpy(buf, tag);
  char c = GPS.read();
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) return;
    strcpy(gps, GPS.lastNMEA());
  }
  else{
    buf[0] = 0;
    gps[0] = 0;
    acc[0] = 0;
    return;
  }
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
  Serial.print(buf);
  File logfile = SD.open(filename, FILE_WRITE);
  if (logfile) {
    logfile.print(buf);
    logfile.close();
  }
  if (strlen(buf) > 200) return;  
  Serial.print(buf);
  delay(10);
  rf95.send((uint8_t *)buf, 200);

//  Serial.println("Waiting for packet to complete..."); 
  delay(10);
  rf95.waitPacketSent();
}
