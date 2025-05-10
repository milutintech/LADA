#include <Wire.h>
#include "TLC59108.h"

#define I2C_ADDR 0x4B  // TLC59108 I2C address (A3:A0 = 1011)

TLC59108 leds(Wire, I2C_ADDR);  // Pass Wire object explicitly

void setup() {
  Serial.begin(115200);
  
  // Initialize Wire on custom pins for ESP32-S3 (SDA = GPIO 1, SCL = GPIO 2)
  Wire.begin(1, 2);

  // Set all 8 channels to max brightness (255 = output LOW → LED ON)
  for (uint8_t channel = 0; channel < 8; channel++) {
    leds.setBrightness(channel, 255);
  }

  Serial.println("All LEDs should be ON (outputs LOW).");
}

void loop() {
  // Nothing needed
}