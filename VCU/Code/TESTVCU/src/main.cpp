#include <Arduino.h>
#include <SPI.h>
#include "mcp2515_can.h"

#define SCK 4
#define MOSI 6
#define MISO 5
#define SPI_CS_PIN 36

SPIClass *customSPI = NULL;
mcp2515_can CAN(SPI_CS_PIN);

void setup() {
    Serial.begin(115200);
    customSPI = new SPIClass(HSPI);
    customSPI->begin(SCK, MISO, MOSI, SPI_CS_PIN);
    CAN.setSPI(customSPI);
    
    while (CAN_OK != CAN.begin(CAN_500KBPS)) {
        Serial.println("CAN Init Failed");
        delay(100);
    }
    Serial.println("CAN Init OK");
}

void loop() {
    static unsigned long lastSend = 0;
    
    if (millis() - lastSend >= 1000) {
        unsigned char testMsg[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
        byte status = CAN.sendMsgBuf(0x100, 0, 8, testMsg);
        Serial.print("Send Status: ");
        Serial.println(status == CAN_OK ? "OK" : "Failed");
        lastSend = millis();
    }

    if(CAN_MSGAVAIL == CAN.checkReceive()) {
        unsigned char len = 0;
        unsigned char buf[8];
        CAN.readMsgBuf(&len, buf);
        unsigned long canId = CAN.getCanId();
        
        Serial.print("Received - ID: 0x");
        Serial.print(canId, HEX);
        Serial.print(" Data: ");
        for(int i = 0; i < len; i++) {
            Serial.print(buf[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }
}