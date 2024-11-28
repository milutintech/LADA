#pragma once
#include "config.h"
#include "vehicle_state.h"
#include <Arduino.h>

class SystemControl {
public:
    SystemControl();
    void armBattery(bool arm);
    void armCoolingSys(bool arm);
    void updateGearState();
    
    bool isPreCharged() const { return HasPrecharged; }
    bool isBatteryArmed() const { return batteryArmed; }
    Gear getCurrentGear() const { return currentGear; }

private:
    unsigned long lastModeChangeTime = 0;
    bool HasPrecharged = false;
    bool batteryArmed = false;
    Gear currentGear = Gear::Neutral;
    bool shiftAttempted = false;
    
    // Temperature thresholds
    const float TEMP_INV_HIGH = 65.0f;
    const float TEMP_MOT_HIGH = 80.0f;
    const float TEMP_INV_LOW = 40.0f;
    const float TEMP_MOT_LOW = 50.0f;
    
    // External sensor values (to be updated via setters)
    float DMC_TempInv = 0;
    float DMC_TempMot = 0;
    float NLG_CoolingRequest = 0;
};