#pragma once
#include "config.h"
#include "vehicle_state.h"

class NLGHandler {
public:
    NLGHandler(mcp2515_can& can);
    void sendNLG();
    void receiveNLG();
    void chargeManage();
    void resetCharging();

private:
    mcp2515_can& canBus;
    byte readDataNLG[MAX_DATA_SIZE] = {0};
    unsigned char controllBufferNLG[8] = {0};

    // Control flags
    bool NLG_C_ClrError = 0;
    bool NLG_C_UnlockConRq = 1;
    bool NLG_C_VentiRq = 0;
    bool NLG_C_EnPhaseShift = 0;
    bool unlockPersist = false;
    
    // State variables
    uint8_t NLG_StateAct = 0;
    uint8_t NLG_StateDem = 0;
    uint16_t NLG_LedDem = 0;
    bool NLG_S_ConLocked = 0;
    uint16_t NLG_DcHvVoltAct = 0;
    uint16_t NLG_DcHvCurrAct = 0;
    
    // Limits
    int NLG_DcHvVoltLimMax = MAX_U_BAT;
    int NLG_DcHvCurrLimMax = MAX_NLG_CURRENT;
    uint16_t NLG_AcCurrLimMax = 32;
    
    void handleSleep();
    void handleStandby();
    void handleReady2Charge();
    void handleCharging();
    unsigned long unlockTimeout = 0;
    const unsigned long UNLOCK_TIMEOUT_DURATION = 3000;
};