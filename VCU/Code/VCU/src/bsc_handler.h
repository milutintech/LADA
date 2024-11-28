#pragma once
#include "config.h"
#include <mcp2515_can.h>

class BSCHandler {
public:
    BSCHandler(mcp2515_can& can);
    void sendBSC();
    void receiveBSC();
    
    float getBSC6_HVVOL_ACT() const { return BSC6_HVVOL_ACT; }
    uint8_t getBSC6_MODE() const { return BSC6_MODE; }
    
private:
    mcp2515_can& canBus;
    byte readDataBSC[MAX_DATA_SIZE] = {0};
    unsigned char controllBufferBSC[8] = {0};
    unsigned char limitBufferBSC[8] = {0};
    
    // BSC State Variables
    float BSC6_HVVOL_ACT = 0;
    float BSC6_LVVOLT_ACT = 0;
    float BSC6_HVCUR_ACT = 0;
    float BSC6_LVCUR_ACT = 0;
    uint8_t BSC6_MODE = 16;
    
    // Control Variables
    bool enableBSC = 0;
    bool modeBSC = 0;
    int Hvoltage = MAX_U_BAT;
    int Lvoltage = 14;
    int LvoltageScale = 0;
    int HvoltageScale = 0;
    
    // Limit Variables
    int BSC6_HVVOL_LOWLIM = MIN_U_BAT;
    int BSC6_HVVOL_LOWLIM_SCALED = 0;
    int BSC6_LVCUR_UPLIM_BUCK = 100;
    int BSC6_HVCUR_UPLIM_BUCK = 20;
    int BSC6_HVCUR_UPLIM_BUCK_SCALED = 0;
};