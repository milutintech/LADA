#include <Arduino.h>
#include <Wire.h>
#include "TLC59108.h"
#include "DisplayController.h"
#include "SpeedometerController.h"
#include "DisplayPatterns.h"
#include "DemoMode.h"
#include "SpeedometerPins.h"
#include "InputController.h"

// Global instances
DisplayController* display = nullptr;
SpeedometerController* speedoController = nullptr;
InputController* inputController = nullptr;
bool demoModeActive = false;
DemoMode* demoMode = nullptr;

// Variables for trip odometer reset
const int TRIP_RESET_PIN = 13;
unsigned long lastTripResetPress = 0;
const unsigned long RESET_DEBOUNCE_TIME = 2000;

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
    
    // Initialize input controller
    inputController = new InputController(0x20);
    if (!inputController->begin()) {
        Serial.println("Failed to initialize MCP23017 input controller!");
    }
    
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
    
    Serial.println("Initialization complete. Running in normal mode.");
    Serial.println("Send 'd' to toggle demo mode.");
    Serial.println("Send 'i' to show input states.");
    Serial.println("Press and hold trip reset button to reset trip odometer.");
}

void handleSerialCommands() {
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        
        switch (cmd) {
            case 'd':
            case 'D':
                toggleDemoMode();
                break;
                
            case 'i':
            case 'I':
                if (inputController) {
                    inputController->printInputStates();
                }
                break;
                
            default:
                Serial.println("Commands: 'd'=demo mode, 'i'=input states");
                break;
        }
    }
}

void loop() {
    // Handle serial commands
    handleSerialCommands();
    
    // Check for trip reset button press
    if (digitalRead(TRIP_RESET_PIN) == LOW) {
        if (lastTripResetPress == 0) {
            lastTripResetPress = millis();
        } else if (millis() - lastTripResetPress >= RESET_DEBOUNCE_TIME) {
            speedoController->resetTripOdometer();
            lastTripResetPress = 0;
        }
    } else {
        lastTripResetPress = 0;
    }
    
    if (demoModeActive) {
        // Run in demo mode
        demoMode->loop();
    } else {
        // Run in normal CAN data mode
        speedoController->processCANMessages();
        
        // Update error lights from mixed input sources
        if (inputController) {
            uint8_t soc = speedoController->getSOC();
            float lvVoltage = speedoController->getLVVoltage();
            bool dmcErrors = speedoController->getDMCHasErrors();
            
            uint16_t errorFlags = inputController->getErrorLightFlags(soc, lvVoltage, dmcErrors);
            speedoController->updateErrorLights(errorFlags);
        }
        
        speedoController->show();
    }
    
    delay(10);
}