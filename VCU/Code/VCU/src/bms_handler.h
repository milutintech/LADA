// bms_handler.h
#pragma once
#include "base_can_handler.h"
#include "message_scaling.h"
#include "vehicle_parameters.h"

class BMSHandler : public BaseCANHandler, protected MessageScaling {
public:
    explicit BMSHandler(mcp2515_can& can);
    void receiveBMS();

    struct BMSData {
        uint8_t soc;
        uint16_t voltage;
        int16_t current;
        uint16_t maxDischarge;
        uint8_t maxCharge;
        bool isValid;
    };

    BMSData getData() const;
    bool isDataValid() const { return dataValid; }

private:
    bool dataValid = false;
    uint8_t BMS_SOC = 0;
    uint16_t BMS_U_BAT = 0;
    int16_t BMS_I_BAT = 0;
    uint16_t BMS_MAX_Discharge = 0;
    uint8_t BMS_MAX_Charge = 0;

    void validateData();
};
