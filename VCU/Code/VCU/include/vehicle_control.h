#pragma once
#include <Arduino.h>
#include "config.h"
#include "vehicle_parameters.h"
#include "ADS1X15.h"

// GearRatio enum moved to config.h to avoid Arduino.h LOW/HIGH conflicts
class VehicleControl {
public:
    explicit VehicleControl(ADS1115& ads);
    
    // Main control functions
    int16_t calculateTorque();
    void setMotorSpeed(float speed);
    void setCurrentGear(GearState gear);
    void setDrivingMode(DriveMode mode);
    bool isDMCEnabled() const;
    
    // Configuration
    void setOPDEnabled(bool enabled) { isOPDEnabled = enabled; }
    void setRegenEnabled(bool enabled) { isRegenEnabled = enabled; }
    void setGearRatio(GearRatio ratio) { currentGearRatio = ratio; }
    
    // Constants
    static constexpr float MAX_VEHICLE_SPEED = 120.0f;  // kph

private:
    // Internal calculation methods
    int32_t samplePedalPosition();
    float calculateVehicleSpeed();
    
    // Driving mode handlers
    int16_t handleLegacyMode(float throttlePosition);
    int16_t handleRegenMode(float throttlePosition, float speed);
    int16_t handleOPDMode(float throttlePosition, float speed);
    
    // OPD mode calculations
    int16_t calculateForwardOPDTorque(float normalizedPosition, float speedPercent);
    int16_t calculateReverseOPDTorque(float normalizedPosition, float speedPercent);
    float calculateRegenSpeedFactor();
    
    // Torque processing
    int16_t applyTorqueLimits(int16_t requestedTorque);
    int16_t applyDeadbandHysteresis(int16_t torque);
    
    // Member variables
    ADS1115& ads;                    // Reference to ADC
    DriveMode currentDrivingMode;    // Current driving mode
    GearState currentGear;           // Current gear state
    GearRatio currentGearRatio;      // Current gear ratio
    
    bool isOPDEnabled;               // OPD mode flag
    bool isRegenEnabled;             // Regeneration enabled flag
    bool enableDMC;                  // DMC enable flag
    bool wasInDeadband;              // Deadband hysteresis state
    bool wasEnabled;                 // Previous enable state
    
    float lastTorque;                // Last calculated torque
    float motorSpeed;                // Current motor speed
};