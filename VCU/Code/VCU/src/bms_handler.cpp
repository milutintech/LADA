// bms_handler.cpp
#include "bms_handler.h"

BMSHandler::BMSHandler(mcp2515_can& can) : BaseCANHandler(can) {}

void BMSHandler::receiveBMS() {
    CANMessage msg;
    if (!receiveMessage(msg)) return;
    
    if (msg.id == 0x001) {
        BMS_SOC = msg.data[0] / 2;
        BMS_U_BAT = 370; // Fixed value as per requirement
        BMS_I_BAT = (msg.data[3] | (msg.data[4] << 8)) / 100;
        BMS_MAX_Discharge = (msg.data[5] | (msg.data[6] << 8)) / 100;
        BMS_MAX_Charge = msg.data[7] * 2;
        validateData();
    }
}

BMSHandler::BMSData BMSHandler::getData() const {
    return {
        BMS_SOC,
        BMS_U_BAT,
        BMS_I_BAT,
        BMS_MAX_Discharge,
        BMS_MAX_Charge,
        dataValid
    };
}

void BMSHandler::validateData() {
    dataValid = BMS_U_BAT >= VehicleParams::Power::MIN_U_BAT && 
                BMS_U_BAT <= VehicleParams::Power::MAX_U_BAT &&
                BMS_SOC <= 100 && 
                BMS_MAX_Discharge <= VehicleParams::Power::MAX_DMC_CURRENT;
}