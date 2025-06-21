#include "InputController.h"

InputController::InputController(uint8_t i2cAddress) : 
    _i2cAddress(i2cAddress), 
    _lastMcpState(0), 
    _lastReadTime(0),
    _initialized(false) {
}

void InputController::setI2CAddress(uint8_t address) {
    if (address >= 0x20 && address <= 0x27) {
        _i2cAddress = address;
        Serial.println("MCP23017 address set to: 0x" + String(address, HEX));
    } else {
        Serial.println("Invalid MCP23017 address! Must be 0x20-0x27");
    }
}

bool InputController::begin() {
    Serial.println("Initializing MCP23017 at address 0x" + String(_i2cAddress, HEX));
    
    // Test if device is present
    Wire.beginTransmission(_i2cAddress);
    if (Wire.endTransmission() != 0) {
        Serial.println("MCP23017 not found at address 0x" + String(_i2cAddress, HEX));
        return false;
    }
    
    // Configure all pins as inputs with pull-ups
    writeRegister(MCP23017_IODIRA, 0xFF);  // All pins in bank A as inputs
    writeRegister(MCP23017_IODIRB, 0xFF);  // All pins in bank B as inputs
    writeRegister(MCP23017_GPPUA, 0xFF);   // Enable pull-ups on bank A
    writeRegister(MCP23017_GPPUB, 0xFF);   // Enable pull-ups on bank B
    
    // Initial read
    _lastMcpState = readMCP23017Inputs();
    _lastReadTime = millis();
    _initialized = true;
    
    Serial.println("MCP23017 initialized successfully");
    Serial.println("Initial input state: 0x" + String(_lastMcpState, HEX));
    
    return true;
}

uint16_t InputController::readMCP23017Inputs() {
    if (!_initialized) return 0;
    
    unsigned long currentTime = millis();
    if (currentTime - _lastReadTime >= READ_INTERVAL) {
        uint8_t bankA = readRegister(MCP23017_GPIOA);
        uint8_t bankB = readRegister(MCP23017_GPIOB);
        
        // Combine both banks into 16-bit value
        _lastMcpState = bankA | (bankB << 8);
        _lastReadTime = currentTime;
    }
    
    return _lastMcpState;
}

bool InputController::getInputState(uint8_t pin, bool activeHigh) {
    if (pin > 15) return false;
    
    uint16_t inputs = readMCP23017Inputs();
    bool pinState = (inputs & (1 << pin)) != 0;
    
    // Return based on active level
    return activeHigh ? pinState : !pinState;
}

uint16_t InputController::getErrorLightFlags(uint8_t soc, float lvVoltage, bool dmcHasErrors) {
    uint16_t errorFlags = 0;
    
    // 0: Parking Brake - MCP23017 input, active low
    if (getInputState(PIN_PARKING_BRAKE, false)) {
        errorFlags |= (1 << ERROR_PARKING_BRAKE);
    }
    
    // 1: Seat Buckle - Not used (always off)
    
    // 2: Battery Light - SOC based (turn on at 20%)
    if (soc <= 20) {
        errorFlags |= (1 << ERROR_BATTERY_LIGHT);
    }
    
    // 3: Diflock - MCP23017 input, active low
    if (getInputState(PIN_DIFLOCK, false)) {
        errorFlags |= (1 << ERROR_DIFLOCK);
    }
    
    // 4: Fluid Low - Not used (always off)
    
    // 5: Brake System - MCP23017 input, active low
    if (getInputState(PIN_BRAKE_SYSTEM, false)) {
        errorFlags |= (1 << ERROR_BRAKE_SYSTEM);
    }
    
    // 6: Battery Low - LV voltage based (turn on below 11V)
    if (lvVoltage < 11.0f) {
        errorFlags |= (1 << ERROR_BATTERY_LOW);
    }
    
    // 7: Indicator - MCP23017 input, active high
    if (getInputState(PIN_INDICATOR, true)) {
        errorFlags |= (1 << ERROR_INDICATOR);
    }
    
    // 8: Normal Lights - MCP23017 input, active high
    if (getInputState(PIN_NORMAL_LIGHTS, true)) {
        errorFlags |= (1 << ERROR_NORMAL_LIGHTS);
    }
    
    // 9: High Beam - MCP23017 input, active high
    if (getInputState(PIN_HIGH_BEAM, true)) {
        errorFlags |= (1 << ERROR_HIGH_BEAM);
    }
    
    // 10: Rear Fog Light - MCP23017 input, active high
    if (getInputState(PIN_REAR_FOG, true)) {
        errorFlags |= (1 << ERROR_REAR_FOG);
    }
    
    // 11: Window Heating - MCP23017 input, active high
    if (getInputState(PIN_WINDOW_HEATING, true)) {
        errorFlags |= (1 << ERROR_WINDOW_HEATING);
    }
    
    // 12: Check Engine - DMC error based
    if (dmcHasErrors) {
        errorFlags |= (1 << ERROR_CHECK_ENGINE);
    }
    
    return errorFlags;
}

void InputController::printInputStates() {
    uint16_t inputs = readMCP23017Inputs();
    
    Serial.println("=== MCP23017 Input States ===");
    Serial.println("Raw value: 0x" + String(inputs, HEX) + " (" + String(inputs, BIN) + ")");
    
    const char* inputNames[] = {
        "Pin00_ParkingBrake(AL)", "Pin01_SeatBuckle(NU)", "Pin02_Spare", "Pin03_Diflock(AL)",
        "Pin04_Spare", "Pin05_BrakeSystem(AL)", "Pin06_Spare", "Pin07_Indicator(AH)",
        "Pin08_NormalLights(AH)", "Pin09_HighBeam(AH)", "Pin10_RearFog(AH)", "Pin11_WindowHeating(AH)",
        "Pin12_Spare", "Pin13_Spare", "Pin14_Spare", "Pin15_Spare"
    };
    
    for (int i = 0; i < 16; i++) {
        bool pinHigh = (inputs & (1 << i)) != 0;
        Serial.printf("%-22s: %s\n", inputNames[i], pinHigh ? "HIGH" : "LOW");
    }
    
    Serial.println("AL=Active Low, AH=Active High, NU=Not Used");
    Serial.println("============================");
}

void InputController::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(_i2cAddress);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

uint8_t InputController::readRegister(uint8_t reg) {
    Wire.beginTransmission(_i2cAddress);
    Wire.write(reg);
    Wire.endTransmission();
    
    Wire.requestFrom(_i2cAddress, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0;
}