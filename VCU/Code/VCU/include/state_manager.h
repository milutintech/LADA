#pragma once
#include <Arduino.h>
#include "config.h"
#include "vehicle_parameters.h"

class StateManager {
public:
    StateManager();
    
    // Main control
    void update();
    void handleWakeup();
    VehicleState getCurrentState() const { return currentState; }
    
    // State transitions
    void transitionToStandby();
    void transitionToRun();
    void transitionToCharging();
    
    // System control
    void armBattery(bool arm);
    void armCoolingSys(bool arm);
    void chargeManage();
    
    // Status methods
    bool isBatteryArmed() const { return batteryArmed; }
    bool isPreCharged() const { return hasPreCharged; }
    bool isCharging() const { return currentState == VehicleState::CHARGING; }
    bool isConnectorLocked() const { return connectorLocked; }
    uint8_t getWakeupReason();
    
    // Configuration
    void setBatteryVoltage(uint16_t voltage) { batteryVoltage = voltage; }
    void setInverterTemp(float temp) { inverterTemp = temp; }
    void setMotorTemp(float temp) { motorTemp = temp; }
    void setCoolingRequest(uint8_t request) { coolingRequest = request; }
    
private:
    // State handlers
    void handleStandbyState();
    void handleRunState();
    void handleChargingState();
    
    // Connector management
    void handleConnectorUnlock();
    void handlePersistentUnlock(unsigned long& unlockTimeout);
    
    // Internal state
    VehicleState currentState;
    bool batteryArmed;
    bool hasPreCharged;
    bool nlgCharged;
    bool connectorLocked;
    bool unlockPersist;
    bool conUlockInterrupt;
    bool enableBSC;
    bool modeBSC;
    bool errorLatch;
    bool enableDMC;
    
    GearState currentGear = GearState::NEUTRAL;  
    bool unlockConnectorRequest = false;         

    // System parameters
    uint16_t batteryVoltage;
    float inverterTemp;
    float motorTemp;
    uint8_t coolingRequest;
    uint8_t chargerState;
    uint8_t chargerStateDemand;
    uint8_t chargeLedDemand;
    float hvVoltageActual;
    uint16_t hvVoltage;
    
    // Timing
    unsigned long lastModeChangeTime;
    unsigned long lastPrechargeAttempt;
};