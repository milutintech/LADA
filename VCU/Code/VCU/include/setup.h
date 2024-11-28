#pragma once
#include <Arduino.h>
#include "state_manager.h"
#include "can_manager.h"
#include "vehicle_control.h"
#include "ADS1X15.h"

class SystemSetup {
public:
    // Main initialization function
    static void initializeSystem(
        ADS1115& ads,
        CANManager& canManager,
        StateManager& stateManager,
        VehicleControl& vehicleControl,
        TaskHandle_t& canTaskHandle,
        TaskHandle_t& controlTaskHandle
    );

private:
    // Hardware initialization
    static void initializeGPIO();
    static void initializeI2C();
    static void initializeADC(ADS1115& ads);
    static void initializeSPI();
    
    // System configuration
    static void initializeSleep();
    static void initializeWatchdog();
    static void setupInterrupts();
    
    // Default pin states
    static void setDefaultPinStates();
    
    // Initial safety checks
    static void performInitialSafetyChecks();
    
    // Private constructor to prevent instantiation
    SystemSetup() = delete;
};