#pragma once
#include "config.h"

class BMSHandler {
public:
    BMSHandler(mcp2515_can& can);
    void receiveBMS();
    
    uint8_t getSOC() const { return BMS_SOC; }
    uint16_t getVoltage() const { return BMS_U_BAT; }
    int16_t getCurrent() const { return BMS_I_BAT; }
    uint16_t getMaxDischarge() const { return BMS_MAX_Discharge; }
    uint8_t getMaxCharge() const { return BMS_MAX_Charge; }

private:
    mcp2515_can& canBus;
    byte readDataBMS[MAX_DATA_SIZE] = {0};
    
    uint8_t BMS_SOC = 0;
    uint16_t BMS_U_BAT = 0;
    int16_t BMS_I_BAT = 0;
    uint16_t BMS_MAX_Discharge = 0;
    uint8_t BMS_MAX_Charge = 0;
};