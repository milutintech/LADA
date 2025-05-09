#include <Arduino.h>
#include <Wire.h>
#include "TLC59108.h"
#include "DisplayController.h"
#include "SpeedometerController.h"
#include "DisplayPatterns.h"
#include "DemoMode.h"

// Global instances
DisplayController* display = nullptr;
SpeedometerController* speedoController = nullptr;
DemoMode* demoMode = nullptr;

void setup() {
    Serial.begin(115200);
    //while (!Serial) delay(10);  // Wait for Serial to be ready
    Serial.println("\nSpeedometer starting up...");
    delay(1000);
    
    Wire.begin(1, 2);  // SDA, SCL pins for ESP32
    
    // Initialize display controller
    display = new DisplayController();
    
    // Initialize speedometer controller
    speedoController = new SpeedometerController();
    if (!speedoController->begin()) {
        Serial.println("Failed to initialize CAN or NeoPixels!");
    }
    
    // Initialize demo mode
    demoMode = new DemoMode(display, speedoController);
}

void loop() {
    demoMode->loop();
}