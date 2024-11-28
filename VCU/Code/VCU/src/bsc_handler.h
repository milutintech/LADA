// bsc_handler.h
#pragma once
#include "base_can_handler.h"
#include "message_scaling.h"
#include "vehicle_parameters.h"

class BSCHandler : public BaseCANHandler, protected MessageScaling {
public:
    explicit BSCHandler(mcp2515_can& can);
    void sendBSC();
    void receiveBSC();

    struct BSCData {
        float hvVoltage;
        float lvVoltage;
        float hvCurrent;
        float lvCurrent;
        uint8_t mode;
    };

    BSCData getData() const;
    void setMode(bool mode) { modeBSC = mode; }
    void enable(bool en) { enableBSC = en; }
    void setVoltages(int hv, int lv) { 
        Hvoltage = hv;
        Lvoltage = lv;
    }

private:
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
    int Hvoltage = VehicleParams::Power::MAX_U_BAT;
    int Lvoltage = 14;
    uint8_t LvoltageScale = 0;
    uint8_t HvoltageScale = 0;
    uint8_t BSC6_HVVOL_LOWLIM_SCALED = 0;
    uint8_t BSC6_HVCUR_UPLIM_BUCK_SCALED = 0;

    void scaleValues();
    void packControlMessage();
    void packLimitMessage();
};