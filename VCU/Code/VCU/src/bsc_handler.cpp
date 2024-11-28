#include "bsc_handler.h"

BSCHandler::BSCHandler(mcp2515_can& can) : canBus(can) {}

void BSCHandler::sendBSC() {
    LvoltageScale = static_cast<uint8_t>(Lvoltage * 10);
    HvoltageScale = static_cast<uint8_t>((Hvoltage - 220));

    // Control message
    controllBufferBSC[0] = (enableBSC << 0) | (modeBSC << 1) | 0x80;
    controllBufferBSC[1] = LvoltageScale;
    controllBufferBSC[2] = HvoltageScale;

    // Limit message
    BSC6_HVVOL_LOWLIM_SCALED = BSC6_HVVOL_LOWLIM - 220;
    BSC6_HVCUR_UPLIM_BUCK_SCALED = static_cast<uint8_t>(BSC6_HVCUR_UPLIM_BUCK * 10);

    limitBufferBSC[0] = BSC6_HVVOL_LOWLIM_SCALED;
    limitBufferBSC[1] = BSC6_LVCUR_UPLIM_BUCK;
    limitBufferBSC[2] = BSC6_HVCUR_UPLIM_BUCK_SCALED;

    canBus.sendMsgBuf(BSC_COMM, 0, 3, controllBufferBSC);
    canBus.sendMsgBuf(BSC_LIM, 0, 6, limitBufferBSC);
}

void BSCHandler::receiveBSC() {
    if (canBus.checkReceive() != CAN_MSGAVAIL) return;

    uint8_t len;
    canBus.readMsgBuf(&len, readDataBSC);
    uint32_t id = canBus.getCanId();

    if (id == 0x26A) {
        BSC6_HVVOL_ACT = ((readDataBSC[0] << 8) | readDataBSC[1]) * 0.1;
        BSC6_LVVOLT_ACT = readDataBSC[2] * 0.1;
        BSC6_HVCUR_ACT = (((readDataBSC[3] << 8) | readDataBSC[4]) * 0.1) - 25;
        BSC6_LVCUR_ACT = ((readDataBSC[5] << 8) | readDataBSC[6]) - 280;
        BSC6_MODE = readDataBSC[7] >> 4;
    }
}