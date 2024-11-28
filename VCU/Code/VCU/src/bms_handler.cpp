#include "bms_handler.h"

BMSHandler::BMSHandler(mcp2515_can& can) : canBus(can) {}

void BMSHandler::receiveBMS() {
    if (canBus.checkReceive() != CAN_MSGAVAIL) return;

    uint8_t len;
    canBus.readMsgBuf(&len, readDataBMS);
    uint32_t id = canBus.getCanId();
    
    if (id == 0x001) {
        BMS_SOC = readDataBMS[0] / 2;
        BMS_U_BAT = 370; // Fixed value as per requirement
        BMS_I_BAT = (readDataBMS[3] | (readDataBMS[4] << 8)) / 100;
        BMS_MAX_Discharge = (readDataBMS[5] | (readDataBMS[6] << 8)) / 100;
        BMS_MAX_Charge = readDataBMS[7] * 2;
        validateData();
    }
}

void BMSHandler::validateData() {
    dataValid = BMS_U_BAT >= MIN_U_BAT && BMS_U_BAT <= MAX_U_BAT &&
                BMS_SOC <= 100 && BMS_MAX_Discharge <= MAX_DMC_CURRENT;
}