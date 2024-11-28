#include "nlg_handler.h"
#include "bms_handler.h"

NLGHandler::NLGHandler(mcp2515_can& can) : canBus(can) {}

void NLGHandler::chargeManage() {
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

void NLGHandler::sendNLG() {
    int NLG_DcHvVoltLimMax_Scale = NLG_DcHvVoltLimMax * 10;
    int NLG_DcHvCurrLimMax_Scale = (NLG_DcHvCurrLimMax + 102.4) * 10;
    int NLG_AcCurrLimMax_Scale = (NLG_AcCurrLimMax + 102.4) * 10;
    
    controllBufferNLG[0] = (NLG_C_ClrError << 7) | (NLG_C_UnlockConRq << 6) | 
                          ((NLG_DcHvVoltLimMax_Scale >> 8) & 0x1F);
    controllBufferNLG[2] = (NLG_StateDem << 5) | 
                          ((NLG_DcHvCurrLimMax_Scale >> 8) & 0x07);
    
    canBus.sendMsgBuf(NLG_DEM_LIM, 0, 8, controllBufferNLG);
}

void NLGHandler::receiveNLG() {
    if (canBus.checkReceive() != CAN_MSGAVAIL) return;
    
    uint8_t len;
    canBus.readMsgBuf(&len, readDataNLG);
    uint32_t id = canBus.getCanId();
    
    if (id == NLG_ACT_LIM) {
        NLG_StateAct = readDataNLG[2] >> 5;
        NLG_S_ConLocked = (readDataNLG[7] >> 5) & 0x01;
        NLG_DcHvVoltAct = ((readDataNLG[0] & 0x1F) << 8) | readDataNLG[1];
        NLG_DcHvCurrAct = ((readDataNLG[2] & 0x07) << 8) | readDataNLG[3];
    }
}

BMSHandler::BMSHandler(mcp2515_can& can) : canBus(can) {}

void BMSHandler::receiveBMS() {
    if (canBus.checkReceive() != CAN_MSGAVAIL) return;
    
    uint8_t len;
    canBus.readMsgBuf(&len, readDataBMS);
    uint32_t id = canBus.getCanId();
    
    if (id == 0x001) {
        BMS_SOC = readDataBMS[0] / 2;
        BMS_U_BAT = 370;  // Fixed value as per original code
        BMS_I_BAT = (readDataBMS[3] | (readDataBMS[4] << 8)) / 100;
        BMS_MAX_Discharge = (readDataBMS[5] | (readDataBMS[6] << 8)) / 100;
        BMS_MAX_Charge = readDataBMS[7] * 2;
    }
}