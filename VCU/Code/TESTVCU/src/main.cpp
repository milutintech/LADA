#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "mcp2515_can.h"
#include "ADS1X15.h"
#include "AD5593R.h"

// Pin definitions from original code
#define SCK 4
#define MOSI 6
#define MISO 5
#define SPI_CS_PIN 36

// Output pins
#define PUMP 38
#define PW1 39
#define CONTACTOR 11
#define NLGKL15 12
#define DMCKL15 13
#define BSCKL15 14
#define BCKLIGHT 17
#define LWP5 18
#define LWP6 21
#define LWP7 16

// Input pins
#define NLG_HW_Wakeup 7
#define IGNITION 8
#define UNLCKCON 9

SPIClass *customSPI = NULL;
mcp2515_can CAN(SPI_CS_PIN);
ADS1115 ADS(0x48);

// Test states
enum TestState {
    TEST_OUTPUTS,
    TEST_INPUTS,
    TEST_ADC,
    TEST_CAN
};

TestState currentTest = TEST_ADC;
unsigned long lastToggle = 0;
unsigned long lastPrint = 0;
bool outputState = false;

void setup() {
    Serial.begin(115200);
    
    // Initialize SPI for CAN
    customSPI = new SPIClass(HSPI);
    customSPI->begin(SCK, MISO, MOSI, SPI_CS_PIN);
    CAN.setSPI(customSPI);
    
    // Initialize I2C
    Wire.begin(1, 2);
    
    // Initialize ADC
    ADS.begin();
    ADS.setGain(2);
    
    // Initialize ADAC
    initADAC(0b1001000, 1, 1);
    setADCpin(0);
    
    // Initialize outputs
    const int outputs[] = {PUMP, PW1, CONTACTOR, NLGKL15, DMCKL15, BSCKL15, BCKLIGHT, LWP5, LWP6, LWP7};
    for(int pin : outputs) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }
    
    // Initialize inputs
    const int inputs[] = {NLG_HW_Wakeup, IGNITION, UNLCKCON};
    for(int pin : inputs) {
        pinMode(pin, INPUT);
    }
    
    // Initialize CAN
    while (CAN_OK != CAN.begin(CAN_500KBPS)) {
        Serial.println("CAN init failed, retrying...");
        delay(100);
    }
    Serial.println("CAN initialized successfully");
}

void testOutputs() {
    if (millis() - lastToggle >= 1000) {
        outputState = !outputState;
        const int outputs[] = {PUMP, PW1, CONTACTOR, NLGKL15, DMCKL15, BSCKL15, BCKLIGHT, LWP5, LWP6, LWP7};
        
        for(int pin : outputs) {
            digitalWrite(pin, outputState);
        }
        
        Serial.print("Outputs set to: ");
        Serial.println(outputState ? "HIGH" : "LOW");
        lastToggle = millis();
    }
}

void testInputs() {
    if (millis() - lastPrint >= 500) {
        const int inputs[] = {NLG_HW_Wakeup, IGNITION, UNLCKCON};
        const char* inputNames[] = {"NLG_HW_Wakeup", "IGNITION", "UNLCKCON"};
        
        Serial.println("\nInput States:");
        for(int i = 0; i < 3; i++) {
            Serial.print(inputNames[i]);
            Serial.print(": ");
            Serial.println(digitalRead(inputs[i]));
        }
        lastPrint = millis();
    }
}

void testADC() {
    if (millis() - lastPrint >= 500) {
        Serial.println("\nADC Readings:");
        for(int i = 0; i < 4; i++) {
            Serial.print("ADC Channel ");
            Serial.print(i);
            Serial.print(": ");
            Serial.println(ADS.readADC(i));
        }
        lastPrint = millis();
    }
}

void testCAN() {
    static unsigned long lastCANSend = 0;
    if (millis() - lastCANSend >= 1000) {
        // Send test message
        unsigned char testMsg[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
        if(CAN.sendMsgBuf(0x100, 0, 8, testMsg) == CAN_OK) {
            Serial.println("CAN message sent successfully");
        } else {
            Serial.println("Error sending CAN message");
        }
        lastCANSend = millis();
    }
    
    // Check for received messages
    if(CAN_MSGAVAIL == CAN.checkReceive()) {
        unsigned char len = 0;
        unsigned char buf[8];
        unsigned long canId = CAN.getCanId();
        
        CAN.readMsgBuf(&len, buf);
        
        Serial.print("Received CAN ID: 0x");
        Serial.print(canId, HEX);
        Serial.print(" Data: ");
        for(int i = 0; i < len; i++) {
            Serial.print(buf[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }
}

void loop() {
    if(Serial.available()) {
        char cmd = Serial.read();
        switch(cmd) {
            case '1':
                currentTest = TEST_OUTPUTS;
                Serial.println("Testing Outputs");
                break;
            case '2':
                currentTest = TEST_INPUTS;
                Serial.println("Testing Inputs");
                break;
            case '3':
                currentTest = TEST_ADC;
                Serial.println("Testing ADC");
                break;
            case '4':
                currentTest = TEST_CAN;
                Serial.println("Testing CAN");
                break;
        }
    }
    
    switch(currentTest) {
        case TEST_OUTPUTS:
            testOutputs();
            break;
        case TEST_INPUTS:
            testInputs();
            break;
        case TEST_ADC:
            testADC();
            break;
        case TEST_CAN:
            testCAN();
            break;
    }
}