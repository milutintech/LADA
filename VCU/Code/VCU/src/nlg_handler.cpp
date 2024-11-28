#include "nlg_handler.h"

NLGHandler::NLGHandler(mcp2515_can& can) : canBus(can) {}

void NLGHandler::sendNLG() {
    int NLG_DcHvVoltLimMax_Scale = static_cast<int>(NLG_DcHvVoltLimMax * 10);
    int NLG_DcHvCurrLimMax_Scale = static_cast<int>((NLG_DcHvCurrLimMax + 102.4) * 10);
    int NLG_AcCurrLimMax_Scale = static_cast<int>((NLG_AcCurrLimMax + 102.4) * 10);

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

    canBus.sendMsgBuf(NLG_DEM_LIM, 0, 8, controllBufferNLG1);
}

void NLGHandler::receiveNLG() {
    if (canBus.checkReceive() != CAN_MSGAVAIL) return;

    uint8_t len;
    canBus.readMsgBuf(&len, readDataNLG);
    uint32_t id = canBus.getCanId();

    switch(id) {
        case NLG_ACT_LIM:
            NLG_StateAct = readDataNLG[2] >> 5;
            NLG_DcHvVoltAct = ((readDataNLG[0] & 0x1F) << 8) | readDataNLG[1];
            NLG_DcHvCurrAct = ((readDataNLG[2] & 0x07) << 8) | readDataNLG[3];
            NLG_S_ConLocked = (readDataNLG[7] >> 5) & 0x01;
            NLG_S_HwWakeup = (readDataNLG[7] >> 3) & 0x01;
            break;

        case NLG_ACT_PLUG:
            NLG_CoolingRequest = static_cast<float>(readDataNLG[4]);
            break;
    }
}

void NLGHandler::chargeManage() {
    if (unlockPersist) {
        if (!NLG_S_ConLocked || (millis() - unlockTimeout > NLG_UNLOCK_TIMEOUT)) {
            unlockPersist = false;
            NLG_C_UnlockConRq = 0;
        }
    }
    handleChargeState();
}

void NLGHandler::handleChargeState() {
    switch (NLG_StateAct) {
        case NLG_ACT_SLEEP:
            NLG_LedDem = 0;
            NLG_StateDem = NLG_DEM_STANDBY;
            break;
            
        case NLG_ACT_STANDBY:
            NLG_LedDem = 1;
            if (NLG_Charged) {
                NLG_StateDem = NLG_DEM_SLEEP;
                NLG_C_UnlockConRq = 1;
                unlockPersist = true;
                unlockTimeout = millis();
            }
            break;
            
        case NLG_ACT_READY2CHARGE:
            NLG_LedDem = 3;
            if (!unlockPersist && HasPrecharged) {
                NLG_StateDem = NLG_DEM_CHARGE;
                NLG_DcHvCurrLimMax = std::min(BMS_MAX_Charge, MAX_NLG_CURRENT);
            }
            break;
            
        case NLG_ACT_CHARGE:
            NLG_LedDem = 4;
            break;
    }
}