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
    int16_t dcCurrent = 0;        // Current for torque display (-450 to +450)
    uint8_t soc = 100;            // State of charge (0-100%)
    uint8_t temperature = 30;     // Temperature (30-110°C)
    uint16_t errorFlags = 0;      // Error indicator bits
    uint8_t currentError = 0;     // Current error to display
    unsigned int speed = 0;       // Vehicle speed (0-200 km/h)
    bool increasing = true;       // Direction of torque change
    bool speedIncreasing = true;  // Direction of speed change
    DisplayController::DriveMode driveMode = DisplayController::MODE_P;
    unsigned long lastModeChange = 0;
    unsigned long modeChangeInterval = 3000; // Change mode every 3 seconds
    uint8_t brightness = 100;     // Illumination brightness
    
    // Kilometer counters
    unsigned long totalKm = 0;
    unsigned long tripKm = 0;
    uint8_t kmDecimal = 0;
};

#endif // DEMO_MODE_H