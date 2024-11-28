#pragma once
#include "config.h"
#include "vehicle_parameters.h"

class BMSHandler {
public:
    BMSHandler(mcp2515_can& can);
    void receiveBMS();
    
    struct BMSData {
        uint8_t soc;
        uint16_t voltage;
        int16_t current;
        uint16_t maxDischarge;
        uint8_t maxCharge;
        bool isValid;
    };

    BMSData getData() const { return {
        BMS_SOC,
        BMS_U_BAT,
        BMS_I_BAT,
        BMS_MAX_Discharge,
        BMS_MAX_Charge,
        dataValid
    }; }

private:
    mcp2515_can& canBus;
    byte readDataBMS[MAX_DATA_SIZE] = {0};
    bool dataValid = false;

    uint8_t BMS_SOC = 0;
    uint16_t BMS_U_BAT = 0;
    int16_t BMS_I_BAT = 0;
    uint16_t BMS_MAX_Discharge = 0;
    uint8_t BMS_MAX_Charge = 0;

    void validateData();
    void processMessage(uint32_t id);
};