
#include <SPI.h>
#include <RH_RF95.h>

//#define RFM95_RST     11   // "A"
//#define RFM95_CS      10   // "B"
//#define RFM95_INT     6    // "D"

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
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

void setup()
{
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  
  Serial.begin(115200);
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
}
char buf[5] = "ping";
int pong = 1;
int dong = 1;
int swtch = 0;
void loop()                     // run over and over again
{
  rf95.send((uint8_t *)buf, 5);
  delay(10);
  rf95.waitPacketSent();
  if(rf95.waitAvailableTimeout(1000)){
    uint8_t got_dat[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(got_dat);
    if (rf95.recv(got_dat, &len)){
      pong = strncmp((char*)got_dat, "PEANUTxxx", 6);
      dong = strncmp((char*)got_dat, "BUTTERxxx", 6);
      if(pong == 0){
        strcpy(buf, "ding");
      }
      else if(dong == 0){
        strcpy(buf, "ping");
      }
      else{
        swtch++;
        if(strcmp(buf, "ping") == 0 && swtch > 3){
          strcpy(buf, "ding");
        }
        else if(strcmp(buf, "ding") == 0 && swtch > 3){
          strcpy(buf, "ping");
        }
      }
      Serial.print((char*)got_dat);
      Serial.flush();
    }
  }
}
