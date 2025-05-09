#ifndef DEMO_MODE_H
#define DEMO_MODE_H

#include <Arduino.h>
#include "DisplayController.h"
#include "SpeedometerController.h"

class DemoMode {
public:
    DemoMode(DisplayController* display, SpeedometerController* speedo);
    void loop();

private:
    DisplayController* display;
    SpeedometerController* speedo;
    unsigned long lastUpdate = 0;
    
    // Demo state variables
    int16_t torque = 0;
    uint8_t soc = 100;
    uint8_t temperature = 30;
    uint16_t errorFlags = 0;
    uint8_t currentError = 0;
    unsigned long speed = 0;
    bool increasing = true;
    DisplayController::DriveMode driveMode = DisplayController::MODE_P;
    unsigned long lastModeChange = 0;
    unsigned long modeChangeInterval = 3000; // Change mode every 3 seconds
    
    // Kilometer counters
    unsigned long totalKm = 0;
    unsigned long tripKm = 0;
    uint8_t kmDecimal = 0;
};

#endif // DEMO_MODE_H