#pragma once
#include "config.h"
#include "vehicle_parameters.h"

class BSCHandler {
public:
    BSCHandler(mcp2515_can& can);
    void sendBSC();
    void receiveBSC();
    
    struct BSCData {
        float hvVoltage;
        float lvVoltage;
        float hvCurrent;
        float lvCurrent;
        uint8_t mode;
    };

    BSCData getData() const { return {
        BSC6_HVVOL_ACT,
        BSC6_LVVOLT_ACT,
        BSC6_HVCUR_ACT,
        BSC6_LVCUR_ACT,
        BSC6_MODE
    }; }

    void setMode(bool mode) { modeBSC = mode; }
    void enable(bool en) { enableBSC = en; }

private:
    mcp2515_can& canBus;
    byte readDataBSC[MAX_DATA_SIZE] = {0};
    unsigned char controllBufferBSC[8] = {0};
    unsigned char limitBufferBSC[8] = {0};

    // State variables
    float BSC6_HVVOL_ACT = 0;
    float BSC6_LVVOLT_ACT = 0;
    float BSC6_HVCUR_ACT = 0;
    float BSC6_LVCUR_ACT = 0;
    uint8_t BSC6_MODE = 16;

    // Control variables
    bool enableBSC = false;
    bool modeBSC = BSC6_BUCK;
    int Hvoltage = MAX_U_BAT;
    int Lvoltage = 14;

    void scaleValues();
    void packControlMessage();
    void packLimitMessage();
};