#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SPI.h>

#define CLIENT_ADDRESS 59
#define SERVER1_ADDRESS 50
#define SERVER2_ADDRESS 42

// for feather m0  
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define LED 13

RH_RF95 driver(RFM95_CS, RFM95_INT);;

RHReliableDatagram manager(driver, CLIENT_ADDRESS);

void setup()
{
  Serial.begin(9600);
  Serial.println("initializing");
  pinMode(LED, OUTPUT);
  if (!manager.init())
    Serial.println("init failed");
    Serial.println("Setting freq");
    driver.setFrequency(915.0);
    driver.setTxPower(23, false);
//    driver.setModemConfig(RH_RF95::Bw31_25Cr48Sf512);
    manager.setTimeout(2000);
}
uint8_t data[] = "Ping";
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
uint8_t SERVERS[2] = {SERVER1_ADDRESS, SERVER2_ADDRESS};
int i = 0;
void loop()
{
  digitalWrite(LED, LOW);
  if (manager.sendtoWait(data, sizeof(data), SERVERS[i]))
  {
    uint8_t len = sizeof(buf);
    uint8_t from;   
    if (manager.recvfromAckTimeout(buf, &len, 2000, &from))
    {
      digitalWrite(LED, HIGH);
      Serial.print((char*)buf);
      Serial.flush();
    }
  }
  else
    Serial.println("JELLY TIME!");
  if (i >= 1){
    i = 0;
  }
  else{
    i++;
  }
}
