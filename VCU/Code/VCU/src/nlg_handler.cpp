// nlg_handler.cpp
#include "nlg_handler.h"

NLGHandler::NLGHandler(mcp2515_can& can) : BaseCANHandler(can) {}

void NLGHandler::sendNLG() {
    packControlMessage();
    sendMessage(NLG_DEM_LIM, 8, controllBufferNLG1);
}

void NLGHandler::receiveNLG() {
    CANMessage msg;
    if (!receiveMessage(msg)) return;
        
    switch(msg.id) {
        case NLG_ACT_LIM:
            processLimitMessage(msg);
            break;
        case NLG_ACT_PLUG:
            NLG_CoolingRequest = static_cast<float>(msg.data[4]);
            break;
    }
}

void NLGHandler::processLimitMessage(const CANMessage& msg) {
    NLG_StateAct = msg.data[2] >> 5;
    NLG_DcHvVoltAct = ((msg.data[0] & 0x1F) << 8) | msg.data[1];
    NLG_DcHvCurrAct = ((msg.data[2] & 0x07) << 8) | msg.data[3];
    NLG_S_ConLocked = (msg.data[7] >> 5) & 0x01;
    NLG_S_HwWakeup = (msg.data[7] >> 3) & 0x01;
}

void NLGHandler::chargeManage() {
    if (unlockPersist) {
        if (!NLG_S_ConLocked || (millis() - unlockTimeout > VehicleParams::Timing::NLG_UNLOCK_TIMEOUT)) {
            unlockPersist = false;
            NLG_C_UnlockConRq = 0;
        }
    }
    handleChargeState();
}

void NLGHandler::handleChargeState() {
    switch (NLG_StateAct) {
        case NLG_ACT_SLEEP:
            handleSleep();
            break;
        case NLG_ACT_STANDBY:
            handleStandby();
            break;
        case NLG_ACT_READY2CHARGE:
            handleReady2Charge();
            break;
        case NLG_ACT_CHARGE:
            handleCharging();
            break;
    }
}

void NLGHandler::handleSleep() {
    NLG_LedDem = 0;
    NLG_StateDem = NLG_DEM_STANDBY;
}

void NLGHandler::handleStandby() {
    NLG_LedDem = 1;
    if (NLG_Charged) {
        NLG_StateDem = NLG_DEM_SLEEP;
        NLG_C_UnlockConRq = 1;
        unlockPersist = true;
        unlockTimeout = millis();
    }
}

void NLGHandler::handleReady2Charge() {
    NLG_LedDem = 3;
    if (unlockPersist || NLG_C_UnlockConRq) {
        NLG_StateDem = NLG_DEM_STANDBY;
    } else if (HasPrecharged) {
        NLG_StateDem = NLG_DEM_CHARGE;
    }
}

void NLGHandler::handleCharging() {
    NLG_LedDem = 4;
}

void NLGHandler::packControlMessage() {
    int NLG_DcHvVoltLimMax_Scale = scaleVoltage(NLG_DcHvVoltLimMax);
    int NLG_DcHvCurrLimMax_Scale = scaleCurrent(NLG_DcHvCurrLimMax);
    int NLG_AcCurrLimMax_Scale = scaleCurrent(NLG_AcCurrLimMax);

    controllBufferNLG1[0] = (NLG_C_ClrError << 7) | 
                           (NLG_C_UnlockConRq << 6) | 
                           ((NLG_DcHvVoltLimMax_Scale >> 8) & 0x1F);
    controllBufferNLG1[1] = NLG_DcHvVoltLimMax_Scale & 0xFF;
    controllBufferNLG1[2] = (NLG_StateDem << 5) | 
                           ((NLG_DcHvCurrLimMax_Scale >> 8) & 0x07);
    controllBufferNLG1[3] = NLG_DcHvCurrLimMax_Scale & 0xFF;
    controllBufferNLG1[4] = (NLG_LedDem << 4) | 
                           ((NLG_AcCurrLimMax_Scale >> 8) & 0x07);
    controllBufferNLG1[5] = NLG_AcCurrLimMax_Scale & 0xFF;
    controllBufferNLG1[6] = 0; // Reserved
    controllBufferNLG1[7] = 0; // Reserved
}