#include <Arduino.h>
#include <Wire.h>
#include "TLC59108.h"

// I2C Pins
#define SDA_PIN 1
#define SCL_PIN 2
#define DIGIT_RESET_1 15
#define DIGIT_RESET_2 16

// Debug flag
#define DEBUG 1

// Simple test with just one TLC59108
TLC59108 *display;

void scanI2C() {
    byte error, address;
    int devices = 0;
 
    Serial.println("Scanning I2C bus...");
 
    for(address = 1; address < 127; address++ ) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
 
        if (error == 0) {
            Serial.printf("I2C device found at address 0x%02X\n", address);
            devices++;
        }
    }
    
    if (devices == 0) {
        Serial.println("No I2C devices found");
    }
}

void setup() {
    // Initialize Serial for debugging
    Serial.begin(115200);
    delay(1000);
    Serial.println("\nStarting Basic TLC59108 Test - Common Cathode Mode");
    
    // Initialize I2C
    Wire.begin(SDA_PIN, SCL_PIN);
    
    // Scan I2C bus
    scanI2C();
    
    // Initialize reset pins
    pinMode(DIGIT_RESET_1, OUTPUT);
    pinMode(DIGIT_RESET_2, OUTPUT);
    
    // Perform hardware reset
    digitalWrite(DIGIT_RESET_1, LOW);
    digitalWrite(DIGIT_RESET_2, LOW);
    delay(1);
    digitalWrite(DIGIT_RESET_1, HIGH);
    digitalWrite(DIGIT_RESET_2, HIGH);
    delay(1);
    
    // Initialize first TLC59108
    byte address = TLC59108::I2C_ADDR::BASE;  // 0x40
    Serial.printf("Initializing TLC59108 at address 0x%02X\n", address);
    
    display = new TLC59108(Wire, address);
    
    // Initialize with no hardware reset pin (we already did it)
    uint8_t initResult = display->init();
    Serial.printf("Init result: %d\n", initResult);
    
    // Set all channels to PWM mode
    uint8_t modeResult = display->setLedOutputMode(TLC59108::LED_MODE::PWM_IND);
    Serial.printf("Set mode result: %d\n", modeResult);
    
    // Turn all segments off initially (0 for common cathode)
    for (int i = 0; i < 8; i++) {
        display->setBrightness(i, 0);
    }
}

void loop() {
    // Basic test pattern - just trying to turn on LEDs
    
    // Test 1: Cycle through each segment individually
    Serial.println("Testing individual segments...");
    for (int segment = 0; segment < 8; segment++) {
        Serial.printf("Testing segment %d\n", segment);
        
        // Turn all segments off
        for (int i = 0; i < 8; i++) {
            display->setBrightness(i, 0);
        }
        
        // Turn on current segment (255 for common cathode)
        display->setBrightness(segment, 255);
        
        // Print debug info
        Serial.printf("Set segment %d to ON (255)\n", segment);
        
        delay(2000);
    }
    
    // Test 2: All segments on
    Serial.println("All segments ON");
    for (int i = 0; i < 8; i++) {
        display->setBrightness(i, 255);
    }
    delay(3000);
    
    // Test 3: All segments off
    Serial.println("All segments OFF");
    for (int i = 0; i < 8; i++) {
        display->setBrightness(i, 0);
    }
    delay(3000);
    
    // Test 4: Brightness ramp on first segment
    Serial.println("Testing brightness ramp on segment 0");
    for (int brightness = 0; brightness <= 255; brightness += 5) {
        display->setBrightness(0, brightness);
        Serial.printf("Segment 0 brightness: %d\n", brightness);
        delay(50);
    }
    delay(1000);
}