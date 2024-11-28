// system_control.h
#pragma once
#include "config.h"
#include "vehicle_parameters.h"
#include "vehicle_state.h"
#include <tuple>

class SystemControl {
public:
    SystemControl() = default;
    
    void armBattery(bool arm) {
        if (arm && !HasPrecharged) {
            handlePrecharge();
        } else {
            disarmBattery();
        }
    }
    
    void armCoolingSys(bool arm) {
        if (batteryArmed && arm && needsCooling()) {
            digitalWrite(PUMP, HIGH);
        } else if (!needsCooling()) {
            digitalWrite(PUMP, LOW);
        }
    }
    
    void updateGearState() {
        auto [forward, reverse] = readGearInputs();
        updateGearBasedOnInputs(forward, reverse);
    }
    
    // Getters
    bool isPreCharged() const { return HasPrecharged; }
    bool isBatteryArmed() const { return batteryArmed; }
    Gear getCurrentGear() const { return currentGear; }
    bool isShiftAttempted() const { return shiftAttempted; }
    
    // Temperature setters used by temperature monitoring system
    void setInverterTemp(float temp) { DMC_TempInv = temp; }
    void setMotorTemp(float temp) { DMC_TempMot = temp; }
    void setCoolingRequest(float request) { NLG_CoolingRequest = request; }
    
    // Battery voltage and current setters
    void setBatteryVoltage(float voltage) { BMS_U_BAT = voltage; }
    void setBatteryCurrent(float current) { BMS_I_BAT = current; }
    
    // Mode and state setters
    void setBSCMode(uint8_t mode) { modeBSC = mode; }
    void setEnableBSC(bool enable) { enableBSC = enable; }
    void setHvoltage(float voltage) { Hvoltage = voltage; }
    
private:
    // State variables
    unsigned long lastModeChangeTime = 0;
    bool HasPrecharged = false;
    bool batteryArmed = false;
    Gear currentGear = Gear::Neutral;
    bool shiftAttempted = false;
    
    // System variables
    float DMC_TempInv = 0;
    float DMC_TempMot = 0;
    float NLG_CoolingRequest = 0;
    float BMS_U_BAT = 0;
    float BMS_I_BAT = 0;
    uint8_t modeBSC = BSC6_BUCK;
    bool enableBSC = false;
    float Hvoltage = 0;
    float BSC6_HVVOL_ACT = 0;

    bool needsCooling() const {
        using namespace VehicleParams;
        return DMC_TempInv > Temperature::INV_HIGH ||
               DMC_TempMot > Temperature::MOT_HIGH ||
               NLG_CoolingRequest > 50;
    }
    
    void handlePrecharge() {
        if (modeBSC != BSC6_BOOST) {
            initializePrecharge();
        } else if (millis() - lastModeChangeTime >= 2000) {
            startPrecharge();
        } else if (isPrechargeComplete()) {
            completePrecharge();
        }
    }
    
    void initializePrecharge() {
        modeBSC = BSC6_BOOST;
        lastModeChangeTime = millis();
        enableBSC = 0;
    }
    
    void startPrecharge() {
        Hvoltage = BMS_U_BAT;
        enableBSC = 1;
    }
    
    bool isPrechargeComplete() const {
        return (BSC6_HVVOL_ACT >= (BMS_U_BAT - 20)) && 
               (BSC6_HVVOL_ACT <= (BMS_U_BAT + 20)) && 
               (BSC6_HVVOL_ACT > 50);
    }
    
    void completePrecharge() {
        HasPrecharged = true;
        batteryArmed = true;
        digitalWrite(CONTACTOR, HIGH);
        enableBSC = 0;
    }
    
    void disarmBattery() {
        batteryArmed = false;
        enableBSC = 0;
        HasPrecharged = false;
        digitalWrite(CONTACTOR, LOW);
    }
    
    std::pair<bool, bool> readGearInputs() {
        bool isForwardHigh = analogRead(GASPEDAL1) > VehicleParams::Transmission::RPM_SHIFT_THRESHOLD;
        bool isReverseHigh = analogRead(GASPEDAL2) > VehicleParams::Transmission::RPM_SHIFT_THRESHOLD;
        return {isForwardHigh, isReverseHigh};
    }
    
    void updateGearBasedOnInputs(bool isForwardHigh, bool isReverseHigh) {
        float currentSpeed = abs(DMC_SpdAct);
        
        if (currentSpeed < VehicleParams::Transmission::RPM_SHIFT_THRESHOLD) {
            updateGearAtLowSpeed(isForwardHigh, isReverseHigh);
        } else {
            handleHighSpeedGearChange(isForwardHigh, isReverseHigh);
        }
    }
    
    void updateGearAtLowSpeed(bool isForwardHigh, bool isReverseHigh) {
        if (isForwardHigh && !isReverseHigh) {
            currentGear = Gear::Drive;
            shiftAttempted = false;
        } else if (!isForwardHigh && isReverseHigh) {
            currentGear = Gear::Reverse;
            shiftAttempted = false;
        } else {
            currentGear = Gear::Neutral;
            shiftAttempted = false;
        }
    }
    
    void handleHighSpeedGearChange(bool isForwardHigh, bool isReverseHigh) {
        if ((isForwardHigh && currentGear == Gear::Reverse) || 
            (isReverseHigh && currentGear == Gear::Drive)) {
            currentGear = Gear::Neutral;
            shiftAttempted = true;
        }
    }
};
