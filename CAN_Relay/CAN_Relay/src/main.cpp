#include <Arduino.h>
#include "mcp2515_can.h"
#include <SPI.h>

//Pinout
#define SCK 6
#define MOSI 5
#define MISO 4
const int SPI_CS_PIN = 7;
const int CAN_INT_PIN = 17;
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin
#define MAX_DATA_SIZE 8
#define RST 4 
#define CAN_ID 0x999

#define Relay1 42
#define Relay2 41
#define Relay3 40
#define Relay4 39
#define Relay5 38
#define Relay6 37

unsigned char controllBufferRelay[8] = {0, 0, 0, 0, 0, 0, 0, 0}; 

uint8_t  len;
uint32_t id;

void setup() {
pinMode(RST, OUTPUT);
digitalWrite(RST, HIGH);
Serial.begin(115200);
while (CAN_OK != CAN.begin(CAN_500KBPS)) {             // init can bus : baudrate = 500k
        Serial.println("CAN BUS Shield init fail");
        delay(100);
    }
pinMode(Relay1, OUTPUT);
pinMode(Relay2, OUTPUT);
pinMode(Relay3, OUTPUT);
pinMode(Relay4, OUTPUT);
pinMode(Relay5, OUTPUT);
pinMode(Relay6, OUTPUT);
  
}

void loop() {
  Serial.println("loop");

  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    
    
    // read data, len: data length, buf: data buf

    CAN.readMsgBuf(&len , controllBufferRelay);

    id = CAN.getCanId();
  
    if(id == CAN_ID){
      Serial.println(controllBufferRelay[0]);
      CAN.sendMsgBuf(0x998, 0, 8, controllBufferRelay);
      digitalWrite(Relay1, controllBufferRelay[0] & 0x01);
      digitalWrite(Relay2, controllBufferRelay[1] & 0x01);
      digitalWrite(Relay3, controllBufferRelay[2] & 0x01);
      digitalWrite(Relay4, controllBufferRelay[3] & 0x01);
      digitalWrite(Relay5, controllBufferRelay[4] & 0x01);
      digitalWrite(Relay6, controllBufferRelay[5] & 0x01);
    } 
  }
}
