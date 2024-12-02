#pragma once
#include <Arduino.h>
#include "can_manager.h"
#include "state_manager.h"
#include "vehicle_control.h"

class SerialConsole {
public:
    SerialConsole(CANManager& canManager, StateManager& stateManager, VehicleControl& vehicleControl);
    void update();
    
private:
    void handleCommand(String command);
    void handleGet(String target, String parameter);
    void handleSet(String target, String parameter, String value);
    void printHelp();
    
    void printValue(const String& name, int value, const String& unit = "");
    void printValue(const String& name, float value, const String& unit = "");
    void printValue(const String& name, bool value);
    
    CANManager& canManager;
    StateManager& stateManager;
    VehicleControl& vehicleControl;
    String inputBuffer;
};