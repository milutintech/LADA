// Simple test to verify a single TLC59108 display on Bus 1
// Upload this to test if ANY display responds on IO1/IO2

#include <Arduino.h>
#include <Wire.h>

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("\n=== Single Display Test ===");

    // Initialize I2C on IO1/IO2
    Wire.begin(1, 2);  // SDA=IO1, SCL=IO2
    Wire.setClock(100000);  // 100kHz

    Serial.println("Scanning for devices on IO1/IO2...");

    for (uint8_t addr = 0x01; addr < 0x7F; addr++) {
        Wire.beginTransmission(addr);
        uint8_t error = Wire.endTransmission();

        if (error == 0) {
            Serial.printf("Device found at 0x%02X\n", addr);

            // Try to read from it
            Wire.requestFrom(addr, (uint8_t)1);
            if (Wire.available()) {
                uint8_t data = Wire.read();
                Serial.printf("  Read byte: 0x%02X\n", data);
            }
        }
    }

    Serial.println("Scan complete.");
}

void loop() {
    delay(5000);
}
