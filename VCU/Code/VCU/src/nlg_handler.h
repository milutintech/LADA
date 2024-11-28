#pragma once
#include "config.h"
#include "vehicle_state.h"
#include "vehicle_parameters.h"

class NLGHandler {
public:
    NLGHandler(mcp2515_can& can);
    void sendNLG();
    void receiveNLG();
    void chargeManage();
    bool isConnectorLocked() const { return NLG_S_ConLocked; }
    uint8_t getState() const { return NLG_StateAct; }
    float getCoolingRequest() const { return NLG_CoolingRequest; }

private:
    mcp2515_can& canBus;
    byte readDataNLG[MAX_DATA_SIZE] = {0};
    unsigned char controllBufferNLG1[8] = {0};

    // State variables
    uint8_t NLG_StateAct = NLG_ACT_SLEEP;
    uint8_t NLG_StateDem = NLG_DEM_STANDBY;
    uint16_t NLG_DcHvVoltAct = 0;
    uint16_t NLG_DcHvCurrAct = 0;
    float NLG_CoolingRequest = 0;
    bool NLG_S_ConLocked = false;
    bool NLG_S_HwWakeup = false;
    bool NLG_Charged = false;

    // Control flags
    bool NLG_C_ClrError = false;
    bool NLG_C_UnlockConRq = true;
    uint16_t NLG_LedDem = 0;

    // Limits
    int NLG_DcHvVoltLimMax = MAX_U_BAT;
    int NLG_DcHvCurrLimMax = MAX_NLG_CURRENT;
    uint16_t NLG_AcCurrLimMax = NLG_MAX_AC_CURRENT;

    unsigned long unlockTimeout = 0;
    bool unlockPersist = false;

    void handleChargeState();
    void updateLimits();
    void processStateTransition();
};