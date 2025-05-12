#include <Arduino.h>
#include <Wire.h>
#include "TLC59108.h"
#include "DisplayController.h"
#include "SpeedometerController.h"
#include "DisplayPatterns.h"
#include "DemoMode.h"
#include "SpeedometerPins.h"  // Include our new pin definitions

// Global instances
DisplayController* display = nullptr;
SpeedometerController* speedoController = nullptr;
bool demoModeActive = false;
DemoMode* demoMode = nullptr;

// Variables for trip odometer reset
const int TRIP_RESET_PIN = 13;  // IO13 for trip reset button
unsigned long lastTripResetPress = 0;
const unsigned long RESET_DEBOUNCE_TIME = 2000; // 2 seconds press to reset

// Function to toggle between demo mode and real data
void toggleDemoMode() {
    demoModeActive = !demoModeActive;
    Serial.println(demoModeActive ? "Demo mode activated" : "Real data mode activated");
}

void setup() {
    Serial.begin(115200);
    Serial.println("\nSpeedometer starting up...");
    delay(1000);
    
    // Initialize trip reset button
    pinMode(TRIP_RESET_PIN, INPUT_PULLUP);
    
    // Initialize I2C with correct pins
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    
    // Initialize display controller
    display = new DisplayController();
    
    // Initialize speedometer controller
    speedoController = new SpeedometerController();
    if (!speedoController->begin()) {
        Serial.println("Failed to initialize CAN or NeoPixels!");
    }
    
    // Initialize demo mode
    demoMode = new DemoMode(display, speedoController);
    
    // Set default illumination brightness
    speedoController->setIllumination(100);
    
    // Initial display will be handled by the odometer values loaded from EEPROM
    
    Serial.println("Initialization complete. Running in normal mode.");
    Serial.println("Send 'd' to toggle demo mode.");
    Serial.println("Press and hold trip reset button to reset trip odometer.");
}

void loop() {
    // Check for serial commands
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        if (cmd == 'd' || cmd == 'D') {
            toggleDemoMode();
        }
    }
    
    // Check for trip reset button press
    if (digitalRead(TRIP_RESET_PIN) == LOW) {
        // Button is pressed
        if (lastTripResetPress == 0) {
            // First detection of press
            lastTripResetPress = millis();
        } else if (millis() - lastTripResetPress >= RESET_DEBOUNCE_TIME) {
            // Button has been pressed for the required time
            speedoController->resetTripOdometer();
            lastTripResetPress = 0;  // Reset to prevent multiple triggers
        }
    } else {
        // Button is released
        lastTripResetPress = 0;
    }
    
    if (demoModeActive) {
        // Run in demo mode
        demoMode->loop();
    } else {
        // Run in normal CAN data mode
        speedoController->processCANMessages();
        speedoController->show();
    }
    
    delay(10); // Small delay to prevent tight looping
}