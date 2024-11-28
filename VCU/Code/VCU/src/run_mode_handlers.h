#pragma once
#include "config.h"
#include "vehicle_state.h"
#include "system_control.h"
#include "dmc_handler.h"
#include "bsc_handler.h"

void handleRunMode() {
    if (!digitalRead(IGNITION)) {
        currentVehicleMode = VehicleMode::Standby;
        return;
    }

    // Enable systems
    sysControl.armCoolingSys(true);
    sysControl.armBattery(true);
    
    // Set KL15 signals
    digitalWrite(DMCKL15, HIGH);
    digitalWrite(BSCKL15, HIGH);
    
    // Process gear state and update reverse light
    sysControl.updateGearState();
    digitalWrite(BCKLIGHT, sysControl.getCurrentGear() == Gear::Reverse ? HIGH : LOW);
    
    // Process DMC torque if battery is precharged
    if (sysControl.isPreCharged()) {
        dmcHandler.processTorque();
    }

    // Update BSC parameters
    if (sysControl.isBatteryArmed()) {
        bscHandler.updateLimits();
    }

    // Monitor temperatures
    if (sysControl.isBatteryArmed()) {
        sysControl.updateCoolingSystem();
    }
}

void handleRunModeCommunication() {
    static const unsigned long FAST_CYCLE = 10;
    static const unsigned long SLOW_CYCLE = 50;
    static unsigned long lastFastCycle = 0;
    static unsigned long lastSlowCycle = 0;
    unsigned long currentTime = millis();

    // Fast cycle tasks (10ms)
    if (currentTime - lastFastCycle >= FAST_CYCLE) {
        dmcHandler.sendDMC();
        lastFastCycle = currentTime;
    }

    // Slow cycle tasks (50ms)
    if (currentTime - lastSlowCycle >= SLOW_CYCLE) {
        bscHandler.sendBSC();
        sysControl.updateGearState();
        lastSlowCycle = currentTime;
    }

    // Continuous polling
    dmcHandler.receiveDMC();
    bscHandler.receiveBSC();
}