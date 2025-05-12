#ifndef SPEEDOMETER_CONTROLLER_H
#define SPEEDOMETER_CONTROLLER_H

#include <Arduino.h>
#include <SPI.h>
#include "mcp2515_can.h"
#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>
#include "SpeedometerPins.h"  // Include our new pin definitions

// NeoPixel section sizes
#define TORQUE_PIXELS 20       // 0-19: torque display
#define SOC_PIXELS 8          // 20-27: state of charge
#define SOC_MARKER_PIXELS 9    // 28-36: SOC markers
#define ERROR_PIXELS 13        // 37-49: error lights
#define TEMP_PIXELS 8         // 50-57: temperature
#define TEMP_MARKER_PIXELS 10  // 58-67: temperature markers
#define ILLUMINATION_PIXELS 30 // 68-97: illumination
#define TOTAL_PIXELS (TORQUE_PIXELS + SOC_PIXELS + SOC_MARKER_PIXELS + ERROR_PIXELS + TEMP_PIXELS + TEMP_MARKER_PIXELS + ILLUMINATION_PIXELS)

// Gear state definitions to match VCU
enum GearState {
    GEAR_NEUTRAL = 0,
    GEAR_DRIVE = 1,
    GEAR_REVERSE = 2
};

// Debug prints
#define DEBUG_PRINTS 1

class SpeedometerController {
public:
    SpeedometerController();
    bool begin();
    void updateTorque(int16_t current);
    void updateSOC(uint8_t percentage);
    void updateErrorLights(uint16_t errorFlags);
    void updateTemperature(uint8_t temp);
    void setIllumination(uint8_t brightness);
    void processCANMessages();
    void show();
    void resetTripOdometer(); // Added for resetting trip meter

private:
    Adafruit_NeoPixel pixels;
    mcp2515_can* canBus;      // Changed to match VCU codebase
    SPIClass* customSPI;      // Added to match VCU implementation
    
    // CAN message data storage
    float torqueAvailable;    // Available torque (Nm)
    float torqueActual;       // Actual torque (Nm)
    float speedActual;        // Motor speed (RPM)
    float dcVoltageAct;       // DC voltage (V)
    float dcCurrentAct;       // DC current (A) - Used for torque display
    float acCurrentAct;       // AC current (A)
    int32_t mechPower;        // Mechanical power (W)
    float tempInverter;       // Inverter temperature (°C)
    float tempMotor;          // Motor temperature (°C)
    int8_t tempSystem;        // System temperature (°C)
    uint8_t soc;              // State of charge (%)
    float bmsVoltage;         // Battery voltage (V)
    int16_t bmsCurrent;       // Battery current (A)
    
    // Derived values
    float vehicleSpeed;       // Calculated vehicle speed (kph)
    float currentSpeed;       // Filtered speed for display (kph)
    GearState gearState;      // Current gear state
    
    // Odometer variables
    float totalOdometer;      // Total distance traveled (km)
    float tripOdometer;       // Trip distance traveled (km)
    unsigned long lastOdometerUpdate;  // Last odometer update time
    unsigned long lastSpeedUpdate;     // Last speed update time
    float lastDistance;                // Last recorded distance for calculation
    float lastSpeedActual;             // Last motor speed for change detection
    
    // Display variables
    int16_t torque;
    uint8_t temperature;
    uint16_t errorFlags;
    
    void clearPixelRange(int start, int count);
    void setPixelRange(int start, int end, uint32_t color);
    void debugPrint(String msg);
    
    // New methods for speedometer functionality
    void updateVehicleSpeed();
    void updateOdometer();
    
    // EEPROM methods for odometer persistence
    void loadOdometersFromEEPROM();
    void saveOdometersToEEPROM();
};

#endif // SPEEDOMETER_CONTROLLER_H