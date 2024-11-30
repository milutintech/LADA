#include "SerialConsole.h"

SerialConsole::SerialConsole(CANManager& canManager, StateManager& stateManager, VehicleControl& vehicleControl)
    : canManager(canManager)
    , stateManager(stateManager)
    , vehicleControl(vehicleControl)
    , inputBuffer("")
{
}

void SerialConsole::update() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (inputBuffer.length() > 0) {
                handleCommand(inputBuffer);
                inputBuffer = "";
            }
        } else {
            inputBuffer += c;
        }
    }
}

void SerialConsole::handleCommand(String command) {
    command.trim();
    command.toLowerCase();
    
    int firstColon = command.indexOf(':');
    int secondColon = command.indexOf(':', firstColon + 1);
    
    if (firstColon == -1) {
        if (command == "help") {
            printHelp();
            return;
        }
        Serial.println("Invalid command format");
        return;
    }
    
    String action = command.substring(0, firstColon);
    
    if (action == "get") {
        if (secondColon == -1) {
            Serial.println("Invalid get command format");
            return;
        }
        String target = command.substring(firstColon + 1, secondColon);
        String parameter = command.substring(secondColon + 1);
        handleGet(target, parameter);
    }
    else if (action == "set") {
        int valueStart = command.indexOf(':', secondColon + 1);
        if (secondColon == -1 || valueStart == -1) {
            Serial.println("Invalid set command format");
            return;
        }
        String target = command.substring(firstColon + 1, secondColon);
        String parameter = command.substring(secondColon + 1, valueStart);
        String value = command.substring(valueStart + 1);
        handleSet(target, parameter, value);
    }
    else {
        Serial.println("Unknown command: " + action);
    }
}

void SerialConsole::handleGet(String target, String parameter) {
    if (target == "nlg") {
        const NLGData& nlg = canManager.getNLGData();
        if (parameter == "state") {
            printValue("NLG State", nlg.stateAct);
        }
        else if (parameter == "voltage") {
            printValue("NLG Voltage", nlg.dcHvVoltageAct / 10.0f, "V");
        }
        else if (parameter == "current") {
            printValue("NLG Current", nlg.dcHvCurrentAct / 10.0f, "A");
        }
        else if (parameter == "temp") {
            printValue("NLG Temperature", nlg.tempCoolPlate, "°C");
        }
        else if (parameter == "all") {
            Serial.println("NLG Status:");
            printValue("State", nlg.stateAct);
            printValue("Voltage", nlg.dcHvVoltageAct / 10.0f, "V");
            printValue("Current", nlg.dcHvCurrentAct / 10.0f, "A");
            printValue("Temperature", nlg.tempCoolPlate, "°C");
            printValue("Connector Locked", nlg.connectorLocked);
        }
    }
    else if (target == "bms") {
        const BMSData& bms = canManager.getBMSData();
        if (parameter == "maxcurrent") {
            printValue("BMS Max Current", bms.maxDischarge, "A");
        }
        else if (parameter == "soc") {
            printValue("BMS SOC", bms.soc, "%");
        }
        else if (parameter == "voltage") {
            printValue("Battery Voltage", bms.voltage, "V");
        }
        else if (parameter == "current") {
            printValue("Battery Current", bms.current, "A");
        }
        else if (parameter == "all") {
            Serial.println("BMS Status:");
            printValue("SOC", bms.soc, "%");
            printValue("Voltage", bms.voltage, "V");
            printValue("Current", bms.current, "A");
            printValue("Max Discharge", bms.maxDischarge, "A");
            printValue("Max Charge", bms.maxCharge, "A");
        }
    }
    else if (target == "dmc") {
        const DMCData& dmc = canManager.getDMCData();
        if (parameter == "motortemp") {
            printValue("Motor Temperature", dmc.tempMotor, "°C");
        }
        else if (parameter == "invertertemp") {
            printValue("Inverter Temperature", dmc.tempInverter, "°C");
        }
        else if (parameter == "status") {
            printValue("Ready", dmc.ready);
            printValue("Running", dmc.running);
        }
        else if (parameter == "all") {
            Serial.println("DMC Status:");
            printValue("Motor Temperature", dmc.tempMotor, "°C");
            printValue("Inverter Temperature", dmc.tempInverter, "°C");
            printValue("System Temperature", dmc.tempSystem, "°C");
            printValue("DC Voltage", dmc.dcVoltageAct, "V");
            printValue("DC Current", dmc.dcCurrentAct, "A");
            printValue("Speed", dmc.speedActual, "rpm");
            printValue("Torque", dmc.torqueActual, "Nm");
            printValue("Ready", dmc.ready);
            printValue("Running", dmc.running);
        }
    }
    else if (target == "vcu") {
        if (parameter == "state") {
            String stateStr;
            switch(stateManager.getCurrentState()) {
                case VehicleState::STANDBY: stateStr = "STANDBY"; break;
                case VehicleState::RUN: stateStr = "RUN"; break;
                case VehicleState::CHARGING: stateStr = "CHARGING"; break;
            }
            Serial.println("VCU State: " + stateStr);
        }
    }
}

void SerialConsole::handleSet(String target, String parameter, String value) {
    if (target == "vcu") {
        String valueLower = value;
        valueLower.toLowerCase();
        bool state = (value == "1" || valueLower == "true");
        
        if (parameter == "bsckl15") {
            digitalWrite(Pins::BSCKL15, state);
            printValue("BSC KL15", state);
        }
        else if (parameter == "dmckl15") {
            digitalWrite(Pins::DMCKL15, state);
            printValue("DMC KL15", state);
        }
        else if (parameter == "nlgkl15") {
            digitalWrite(Pins::NLGKL15, state);
            printValue("NLG KL15", state);
        }
        else if (parameter == "pump") {
            digitalWrite(Pins::PUMP, state);
            printValue("Cooling Pump", state);
        }
    }
}

void SerialConsole::printHelp() {
    Serial.println("Available commands:");
    Serial.println("\nGet commands:");
    Serial.println("  get:nlg:[state|voltage|current|temp|all]");
    Serial.println("  get:bms:[soc|voltage|current|maxcurrent|all]");
    Serial.println("  get:dmc:[motortemp|invertertemp|status|all]");
    Serial.println("  get:vcu:state");
    Serial.println("\nSet commands:");
    Serial.println("  set:vcu:bsckl15:[0|1]");
    Serial.println("  set:vcu:dmckl15:[0|1]");
    Serial.println("  set:vcu:nlgkl15:[0|1]");
    Serial.println("  set:vcu:pump:[0|1]");
    Serial.println("\nOther commands:");
    Serial.println("  help - Show this help message");
}

void SerialConsole::printValue(const String& name, int value, const String& unit) {
    Serial.print(name + ": ");
    Serial.print(value);
    if (unit.length() > 0) {
        Serial.print(unit);
    }
    Serial.println();
}

void SerialConsole::printValue(const String& name, float value, const String& unit) {
    Serial.print(name + ": ");
    Serial.print(value, 1);
    if (unit.length() > 0) {
        Serial.print(unit);
    }
    Serial.println();
}

void SerialConsole::printValue(const String& name, bool value) {
    Serial.println(name + ": " + (value ? "True" : "False"));
}