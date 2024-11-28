// nlg_handler.h
#pragma once
#include "base_can_handler.h"
#include "message_scaling.h"
#include "vehicle_parameters.h"

class NLGHandler : public BaseCANHandler, protected MessageScaling {
public:
    explicit NLGHandler(mcp2515_can& can);
    void sendNLG();
    void receiveNLG();
    void chargeManage();
    void resetCharging();

    struct NLGData {
        uint8_t state;
        uint16_t voltage;
        uint16_t current;
        float coolingRequest;
        bool connectorLocked;
        bool hwWakeup;
    };

    NLGData getData() const;
    bool isConnectorLocked() const { return NLG_S_ConLocked; }
    uint8_t getState() const { return NLG_StateAct; }
    float getCoolingRequest() const { return NLG_CoolingRequest; }

private:
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
    int NLG_DcHvVoltLimMax = VehicleParams::Power::MAX_U_BAT;
    int NLG_DcHvCurrLimMax = VehicleParams::Power::MAX_NLG_CURRENT;
    uint16_t NLG_AcCurrLimMax = VehicleParams::Power::NLG_MAX_AC;

    unsigned long unlockTimeout = 0;
    bool unlockPersist = false;
    bool HasPrecharged = false;

    void handleChargeState();
    void processLimitMessage(const CANMessage& msg);
    void handleSleep();
    void handleStandby();
    void handleReady2Charge();
    void handleCharging();
    void packControlMessage();
};
