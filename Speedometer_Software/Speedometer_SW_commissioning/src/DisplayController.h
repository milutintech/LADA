#ifndef DISPLAY_CONTROLLER_H
#define DISPLAY_CONTROLLER_H

#include <Arduino.h>
#include <Wire.h>
#include "TLC59108.h"
#include "DisplayPatterns.h"

#define NUM_ICS 15
#define TOTAL_KM_START 0
#define TRIP_KM_START 6
#define SPEED_START 10
#define MODE_SEGMENT_START 13

class DisplayController {
public:
    enum DriveMode {
        MODE_P = 0, MODE_R = 1, MODE_N = 2, MODE_D = 3, MODE_S = 4
    };

    DisplayController();
    ~DisplayController();
    
    void displayTotalKm(unsigned long km);
    void displayTripKm(unsigned long km, uint8_t decimal);
    void displaySpeed(unsigned int speed);
    void displayDriveMode(DriveMode mode);
    void clear();

private:
    TLC59108** displays;
    byte* icAddresses;     // Array to store the actual I2C addresses for each display
    bool* validDisplays;   // Array to track which displays are valid/present
    void setSevenSegment(uint8_t icIndex, uint8_t digit, bool showDot = false);
};

extern DisplayController* display;

#endif // DISPLAY_CONTROLLER_H