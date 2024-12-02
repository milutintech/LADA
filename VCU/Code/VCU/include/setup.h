#pragma once
#include <Arduino.h>
#include "state_manager.h"
#include "can_manager.h"
#include "vehicle_control.h"
#include "ADS1X15.h"

class SystemSetup {
public:
    static void initializeGPIO();
    static void initializeSleep();

private:
    static void setDefaultPinStates();
    
    // Private constructor to prevent instantiation
    SystemSetup() = delete;
};