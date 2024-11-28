// bsc_handler.cpp
#include "bsc_handler.h"

BSCHandler::BSCHandler(mcp2515_can& can) : BaseCANHandler(can) {}

void BSCHandler::sendBSC() {
    scaleValues();
    packControlMessage();
    packLimitMessage();
    
    sendMessage(BSC_COMM, 3, controllBufferBSC);
    sendMessage(BSC_LIM, 6, limitBufferBSC);
}

void BSCHandler::receiveBSC() {
    CANMessage msg;
    if (!receiveMessage(msg)) return;

    if (msg.id == 0x26A) {
        BSC6_HVVOL_ACT = unscaleVoltage((msg.data[0] << 8) | msg.data[1]);
        BSC6_LVVOLT_ACT = unscaleVoltage(msg.data[2]);
        BSC6_HVCUR_ACT = unscaleCurrent((msg.data[3] << 8) | msg.data[4]);
        BSC6_LVCUR_ACT = static_cast<float>((msg.data[5] << 8) | msg.data[6] - 280);
        BSC6_MODE = msg.data[7] >> 4;
    }
}

BSCHandler::BSCData BSCHandler::getData() const {
    return {
        BSC6_HVVOL_ACT,
        BSC6_LVVOLT_ACT,
        BSC6_HVCUR_ACT,
        BSC6_LVCUR_ACT,
        BSC6_MODE
    };
}

void BSCHandler::scaleValues() {
    LvoltageScale = static_cast<uint8_t>(Lvoltage * 10);
    HvoltageScale = static_cast<uint8_t>((Hvoltage - 220));
    BSC6_HVVOL_LOWLIM_SCALED = VehicleParams::Power::MIN_U_BAT - 220;
    BSC6_HVCUR_UPLIM_BUCK_SCALED = static_cast<uint8_t>(VehicleParams::Power::BSC_LV_BUCK * 10);
}

void BSCHandler::packControlMessage() {
    controllBufferBSC[0] = (enableBSC << 0) | (modeBSC << 1) | 0x80;
    controllBufferBSC[1] = LvoltageScale;
    controllBufferBSC[2] = HvoltageScale;
}

void BSCHandler::packLimitMessage() {
    limitBufferBSC[0] = BSC6_HVVOL_LOWLIM_SCALED;
    limitBufferBSC[1] = VehicleParams::Power::BSC_LV_BUCK;
    limitBufferBSC[2] = BSC6_HVCUR_UPLIM_BUCK_SCALED;
}