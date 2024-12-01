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
            printDetailedState();
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

void SerialConsole::printDetailedState() {
    VehicleState currentState = stateManager.getCurrentState();
    const BMSData& bms = canManager.getBMSData();
    const BSCData& bsc = canManager.getBSCData();
    const DMCData& dmc = canManager.getDMCData();
    const NLGData& nlg = canManager.getNLGData();

    // Print main state
    switch(currentState) {
        case VehicleState::STANDBY:
            Serial.println("VCU Main State: STANDBY");
            printValue("Wake Signal", digitalRead(Pins::IGNITION) || digitalRead(Pins::NLG_HW_Wakeup));
            printValue("Battery Voltage", bms.voltage, "V");
            break;
            
        case VehicleState::RUN:
            Serial.println("VCU Main State: RUN");
            printRunStateDetails(bms, dmc);
            break;
            
        case VehicleState::CHARGING:
            Serial.println("VCU Main State: CHARGING");
            printChargingStateDetails(bms, nlg);
            break;
    }

    // Print sub-states and system status
    if (!stateManager.isPreCharged() && stateManager.isBatteryArmed()) {
        printPrechargeDetails(bms, bsc);
    }

    // Print error states if any
    printErrorStates(bms, dmc, nlg);

    // Print power train status
    printPowerTrainStatus(bms, dmc);

    // Print thermal management status
    printThermalStatus(dmc, nlg);
}

void SerialConsole::printRunStateDetails(const BMSData& bms, const DMCData& dmc) {
    Serial.println("\nRun State Details:");
    printValue("Battery Armed", stateManager.isBatteryArmed());
    printValue("Precharge Complete", stateManager.isPreCharged());
    printValue("DMC Ready", dmc.ready);
    printValue("DMC Running", dmc.running);
    printValue("Battery SOC", bms.soc, "%");
    printValue("Available Discharge Current", bms.maxDischarge, "A");
    printValue("Motor Speed", dmc.speedActual, "rpm");
    printValue("Motor Torque", dmc.torqueActual, "Nm");
}

void SerialConsole::printChargingStateDetails(const BMSData& bms, const NLGData& nlg) {
    Serial.println("\nCharging State Details:");
    printValue("SOC", bms.soc, "%");
    printValue("Charging Current", nlg.dcHvCurrentAct / 10.0f, "A");
    printValue("Battery Voltage", bms.voltage, "V");
    printValue("Connector Locked", nlg.connectorLocked);
    printValue("Max Charging Current", bms.maxCharge, "A");
    printValue("Charger State", nlg.stateAct);
    printValue("Control Pilot State", nlg.stateCtrlPilot);
}

void SerialConsole::printPrechargeDetails(const BMSData& bms, const BSCData& bsc) {
    Serial.println("\nPrecharge Status:");
    printValue("Battery Voltage", bms.voltage, "V");
    printValue("HV Bus Voltage", bsc.hvVoltageAct, "V");
    float progress = (bsc.hvVoltageAct / bms.voltage) * 100.0f;
    printValue("Precharge Progress", progress, "%");
    printValue("BSC Mode", bsc.mode == BSCModes::BSC6_BOOST ? "BOOST" : "BUCK");
}

void SerialConsole::printErrorStates(const BMSData& bms, const DMCData& dmc, const NLGData& nlg) {
    bool hasErrors = false;
    Serial.println("\nSystem Errors:");
    
    if (bms.voltage < VehicleParams::Battery::MIN_VOLTAGE) {
        Serial.println("ERROR: Battery voltage too low");
        hasErrors = true;
    }
    if (dmc.tempMotor > VehicleParams::Temperature::MOT_HIGH) {
        Serial.println("ERROR: Motor temperature too high");
        hasErrors = true;
    }
    if (dmc.tempInverter > VehicleParams::Temperature::INV_HIGH) {
        Serial.println("ERROR: Inverter temperature too high");
        hasErrors = true;
    }
    
    if (!hasErrors) {
        Serial.println("No active errors");
    }
}

void SerialConsole::printPowerTrainStatus(const BMSData& bms, const DMCData& dmc) {
    Serial.println("\nPowertrain Status:");
    printValue("Battery Voltage", bms.voltage, "V");
    printValue("Battery Current", bms.current, "A");
    printValue("Motor Power", dmc.mechPower, "W");
    printValue("Inverter Efficiency", 
              dmc.mechPower != 0 ? (dmc.dcVoltageAct * dmc.dcCurrentAct / dmc.mechPower) * 100 : 0, 
              "%");
}

void SerialConsole::printThermalStatus(const DMCData& dmc, const NLGData& nlg) {
    Serial.println("\nThermal Management:");
    printValue("Motor Temperature", dmc.tempMotor, "°C");
    printValue("Inverter Temperature", dmc.tempInverter, "°C");
    printValue("Charger Temperature", nlg.tempCoolPlate, "°C");
    printValue("Cooling Request", nlg.coolingRequest, "%");
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