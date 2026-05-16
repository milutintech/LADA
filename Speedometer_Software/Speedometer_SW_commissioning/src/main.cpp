#include <Arduino.h>
#include <Wire.h>
#include "TLC59108.h"
#include "DisplayController.h"
#include "SpeedometerController.h"
#include "DisplayPatterns.h"
#include "DemoMode.h"
#include "SpeedometerPins.h"
#include "InputController.h"
#include "Globals.h"

// Global instances
DisplayController* display = nullptr;
SpeedometerController* speedoController = nullptr;
InputController* inputController = nullptr;
bool demoModeActive = false;
DemoMode* demoMode = nullptr;
uint8_t verboseLevel = VERBOSE_NORMAL;  // Default to normal verbosity

// Global brightness settings - can be adjusted at runtime
uint8_t brightness7Segment = 50;        // 7-segment displays (Speed, Total KM, Trip KM)
uint8_t brightnessDriveMode = 50;       // PNDRS 14-segment display
uint8_t brightnessNeoPixelGlobal = 100; // Global NeoPixel brightness (applied to all pixels)
uint8_t brightnessMarkers = 255;        // Marker LEDs brightness multiplier
uint8_t brightnessDataBars = 255;       // Data bar brightness multiplier (Torque, SOC, Temp)
uint8_t brightnessErrorLights = 255;    // Error/status lights brightness multiplier

// Variables for trip odometer reset
const int TRIP_RESET_PIN = 13;
unsigned long lastTripResetPress = 0;
const unsigned long RESET_DEBOUNCE_TIME = 2000;

// Function to cycle through verbose levels
void cycleVerboseLevel() {
    verboseLevel = (verboseLevel + 1) % 3;  // Cycle 0->1->2->0

    const char* levelNames[] = {"SILENT", "NORMAL", "DEBUG"};
    Serial.printf("Verbose level: %d (%s)\n", verboseLevel, levelNames[verboseLevel]);

    if (verboseLevel == VERBOSE_SILENT) {
        Serial.println("  - Errors only");
    } else if (verboseLevel == VERBOSE_NORMAL) {
        Serial.println("  - Important events (mode changes, warnings)");
    } else {
        Serial.println("  - All messages (init + runtime updates)");
    }
}

// Function to toggle between demo mode and real data
void toggleDemoMode() {
    demoModeActive = !demoModeActive;
    Serial.println(demoModeActive ? "Demo mode activated" : "Real data mode activated");
}

// I2C bus scanner to help diagnose display issues
void scanI2CBus(TwoWire& wire, const char* busName) {
    Serial.printf("\n=== Scanning %s ===\n", busName);
    byte error, address;
    int devicesFound = 0;

    for (address = 1; address < 127; address++) {
        wire.beginTransmission(address);
        error = wire.endTransmission();

        if (error == 0) {
            Serial.printf("Device found at 0x%02X\n", address);
            devicesFound++;
        }
    }

    if (devicesFound == 0) {
        Serial.printf("No I2C devices found on %s\n", busName);
    } else {
        Serial.printf("Found %d device(s) on %s\n", devicesFound, busName);
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("\nSpeedometer starting up...");
    delay(1000);
    
    // Initialize trip reset button (active HIGH)
    pinMode(TRIP_RESET_PIN, INPUT);

    // Initialize I2C Bus 1 (Total KM + Trip KM displays)
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(200000);  // 200kHz - balance between reliability and speed
    Wire.setTimeout(50);    // Set 50ms timeout to prevent blocking

    // Send a general call reset to all I2C devices on Bus 1
    Wire.beginTransmission(0x00);  // General call address
    Wire.write(0x06);              // Software reset command
    Wire.endTransmission();
    delay(10);  // Allow devices to reset

    DEBUG_PRINTLN("I2C Bus 1 initialized on SDA=IO1, SCL=IO2 @ 200kHz");

    // Initialize I2C Bus 2 (Speed + Drive Mode displays)
    Wire1.begin(I2C2_SDA_PIN, I2C2_SCL_PIN);
    Wire1.setClock(400000);  // Standard 400kHz for Bus 2 (works fine with fewer devices)
    Wire1.setTimeout(50);   // Set 50ms timeout to prevent blocking
    DEBUG_PRINTLN("I2C Bus 2 initialized on SDA=IO19, SCL=IO20 @ 400kHz");

    // Initialize input controller
    inputController = new InputController(0x20);
    if (!inputController->begin()) {
        DEBUG_PRINTLN("Failed to initialize MCP23017 input controller!");
    }

    // Initialize display controller
    display = new DisplayController();

    // Initialize speedometer controller
    speedoController = new SpeedometerController();
    if (!speedoController->begin()) {
        DEBUG_PRINTLN("Failed to initialize CAN or NeoPixels!");
    }
    
    // Initialize demo mode
    demoMode = new DemoMode(display, speedoController);
    
    // Set default illumination brightness
    speedoController->setIllumination(100);
    
    Serial.println("Initialization complete. Running in normal mode.");
    Serial.println("Commands:");
    Serial.println("  'd' - Toggle demo mode");
    Serial.println("  'i' - Show input states");
    Serial.println("  's' - Scan I2C buses");
    Serial.println("  'v' - Cycle verbose level (Silent/Normal/Debug)");
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

            case 's':
            case 'S':
                scanI2CBus(Wire, "I2C Bus 1 (SDA=IO1, SCL=IO2)");
                scanI2CBus(Wire1, "I2C Bus 2 (SDA=IO19, SCL=IO20)");
                break;

            case 'v':
            case 'V':
                cycleVerboseLevel();
                break;

            default:
                Serial.println("Commands: 'd'=demo, 'i'=inputs, 's'=scan I2C, 'v'=verbose");
                break;
        }
    }
}

void loop() {
    // Handle serial commands
    handleSerialCommands();
    
    // Check for trip reset button press (active HIGH)
    if (digitalRead(TRIP_RESET_PIN) == HIGH) {
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
    }

    // Always call show() to update NeoPixels (both demo and normal mode)
    speedoController->show();
    
    delay(10);
}