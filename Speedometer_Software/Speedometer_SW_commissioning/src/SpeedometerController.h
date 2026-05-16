#ifndef SPEEDOMETER_CONTROLLER_H
#define SPEEDOMETER_CONTROLLER_H

#include <Arduino.h>
#include <SPI.h>
#include "mcp2515_can.h"
#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>
#include "SpeedometerPins.h"  // Include our new pin definitions

// NeoPixel section sizes
#define TORQUE_PIXELS 37       // 0-36: torque display (LED 6 is zero/center)
#define SOC_PIXELS 16          // 37-52: state of charge
#define SOC_MARKER_PIXELS 10   // 53-62: SOC markers (100%, 75%, 50%, 25%, E)
#define ERROR_PIXELS 13        // 63-75: error/status lights
#define TEMP_PIXELS 16         // 76-91: temperature
#define TEMP_MARKER_PIXELS 10  // 92-101: temperature markers (30°C, 50°C, 70°C, 90°C, 110°C)
#define ILLUMINATION_PIXELS 0  // No illumination LEDs
#define TOTAL_PIXELS (TORQUE_PIXELS + SOC_PIXELS + SOC_MARKER_PIXELS + ERROR_PIXELS + TEMP_PIXELS + TEMP_MARKER_PIXELS + ILLUMINATION_PIXELS)

// Gear state definitions to match VCU
enum GearState {
    GEAR_NEUTRAL = 0,
    GEAR_DRIVE = 1,
    GEAR_REVERSE = 2
};

// Debug prints - Set to 0 to disable runtime debug messages (they're very slow!)
#define DEBUG_PRINTS 0

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
    void resetTripOdometer();
    
    // Getter methods for InputController
    uint8_t getSOC() const { return soc; }
    float getLVVoltage() const { return lvVoltageAct; }
    bool getDMCHasErrors() const { return dmcHasErrors; }

    // Getter and setter methods for odometer (used by demo mode)
    float getTotalOdometer() const { return totalOdometer; }
    float getTripOdometer() const { return tripOdometer; }
    void addDistance(float km);

private:
    Adafruit_NeoPixel pixels;
    mcp2515_can* canBus;
    SPIClass* customSPI;
    
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
    
    // BSC data from 0x26A
    float hvVoltageAct;       // HV voltage (V)
    float lvVoltageAct;       // LV voltage (V) - Used for battery low warning
    float hvCurrentAct;       // HV current (A)
    float lvCurrentAct;       // LV current (A)
    
    // DMC error flags from 0x25A
    bool dmcHasErrors;        // True if any DMC error is active
    
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
    
    // Color helper functions for fading effects
    uint32_t hslToRgb(float h, float s, float l);
    uint32_t getTemperatureColor(uint8_t temp);
    uint32_t getSOCColor(uint8_t percentage);

    // Brightness multiplier helpers
    uint32_t applyBrightness(uint32_t color, uint8_t multiplier);
};

#endif // SPEEDOMETER_CONTROLLER_H