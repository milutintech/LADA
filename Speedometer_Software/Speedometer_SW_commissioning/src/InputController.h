#ifndef INPUT_CONTROLLER_H
#define INPUT_CONTROLLER_H

#include <Arduino.h>
#include <Wire.h>

// MCP23017 Register addresses
#define MCP23017_IODIRA   0x00
#define MCP23017_IODIRB   0x01
#define MCP23017_GPPUA    0x0C
#define MCP23017_GPPUB    0x0D
#define MCP23017_GPIOA    0x12
#define MCP23017_GPIOB    0x13

// Input pin definitions for MCP23017
enum InputPins {
    PIN_PARKING_BRAKE = 0,    // Active low
    PIN_SEAT_BUCKLE = 1,      // Not used
    PIN_DIFLOCK = 3,          // Active low
    PIN_BRAKE_SYSTEM = 5,     // Active low
    PIN_INDICATOR = 7,        // Active high
    PIN_NORMAL_LIGHTS = 8,    // Active high  
    PIN_HIGH_BEAM = 9,        // Active high
    PIN_REAR_FOG = 10,        // Active high
    PIN_WINDOW_HEATING = 11   // Active high
};

// Error light bit positions (matching your existing code)
enum ErrorLights {
    ERROR_PARKING_BRAKE = 0,
    ERROR_SEAT_BUCKLE = 1,     // Not used
    ERROR_BATTERY_LIGHT = 2,   // SOC based
    ERROR_DIFLOCK = 3,
    ERROR_FLUID_LOW = 4,       // Not used
    ERROR_BRAKE_SYSTEM = 5,
    ERROR_BATTERY_LOW = 6,     // LV voltage based
    ERROR_INDICATOR = 7,
    ERROR_NORMAL_LIGHTS = 8,
    ERROR_HIGH_BEAM = 9,
    ERROR_REAR_FOG = 10,
    ERROR_WINDOW_HEATING = 11,
    ERROR_CHECK_ENGINE = 12    // DMC error based
};

class InputController {
public:
    InputController(uint8_t i2cAddress = 0x20);
    bool begin();
    uint16_t getErrorLightFlags(uint8_t soc, float lvVoltage, bool dmcHasErrors);
    void setI2CAddress(uint8_t address);
    void printInputStates();
    uint16_t readMCP23017Inputs();

private:
    uint8_t _i2cAddress;
    uint16_t _lastMcpState;
    unsigned long _lastReadTime;
    const unsigned long READ_INTERVAL = 50; // Read every 50ms
    
    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
    bool _initialized;
    
    // Helper functions
    bool getInputState(uint8_t pin, bool activeHigh = false);
};

#endif // INPUT_CONTROLLER_H