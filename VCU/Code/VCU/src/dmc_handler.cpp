#include "dmc_handler.h"

DMCHandler::DMCHandler(mcp2515_can& can) : canBus(can) {}

void DMCHandler::sendDMC() {
    DMC_TrqRq_Scale = calculateTorque();
    
    uint8_t controlByte = (enableDMC << 7) | (modeDMC << 6) | 
                         (1 << 5) | (1 << 1) | 1;
    
    controllBufferDMC[0] = controlByte;
    controllBufferDMC[2] = DMC_SpdRq >> 8;
    controllBufferDMC[3] = DMC_SpdRq & 0xFF;
    controllBufferDMC[4] = (DMC_TrqRq_Scale * 10) >> 8;
    controllBufferDMC[5] = (DMC_TrqRq_Scale * 10) & 0xFF;
    
    scaleAndPackValues();
    
    canBus.sendMsgBuf(DMCCTRL, 0, 8, controllBufferDMC);
    canBus.sendMsgBuf(DMCLIM, 0, 8, limitBufferDMC);
    canBus.sendMsgBuf(DMCCTRL2, 0, 8, controllBuffer2DMC);
}

void DMCHandler::receiveDMC() {
    if (canBus.checkReceive() != CAN_MSGAVAIL) return;

    uint8_t len;
    canBus.readMsgBuf(&len, readDataDMC);
    uint32_t id = canBus.getCanId();
    
    switch(id) {
        case 0x258:
            DMC_Ready = readDataDMC[0] & 0x80;
            DMC_Running = readDataDMC[0] & 0x40;
            DMC_SpdAct = static_cast<int16_t>((readDataDMC[6] << 8) | readDataDMC[7]);
            break;
            
        case 0x458:
            DMC_TempInv = ((readDataDMC[0] << 8) | readDataDMC[1]) * 0.5f;
            DMC_TempMot = ((readDataDMC[2] << 8) | readDataDMC[3]) * 0.5f;
            break;
            
        case 0x259:
            DMC_DcVltAct = ((readDataDMC[0] << 8) | readDataDMC[1]) * 0.1f;
            DMC_DcCurrAct = ((readDataDMC[2] << 8) | readDataDMC[3]) * 0.1f;
            DMC_MechPwr = ((readDataDMC[6] << 8) | readDataDMC[7]) * 16;
            break;
    }
}

float DMCHandler::calculateRegenTorque(float speed, float throttle) {
    if (abs(speed) < ZERO_SPEED_WINDOW) return 0;
    
    if (speed < -MIN_SPEED_FOR_REGEN) {
        if (throttle < REGEN_END_POINT) {
            float regenFactor = 1.0f - (throttle / REGEN_END_POINT);
            regenFactor = pow(regenFactor, 1.8f);
            float speedFactor = (abs(speed) > 1000.0f) ? 1.2f : 1.0f;
            return regenFactor * (DMC_MAXREQTRQ * 0.7f) * speedFactor;
        }
        if (throttle < COAST_END_POINT) return 0;
    }
    
    return calculateDriveTorque(throttle, false);
}

float DMCHandler::calculateDriveTorque(float throttle, bool reverse) {
    if (throttle <= 5.0f) return 0;
    
    float maxTorque = reverse ? MAX_REVERSE_TRQ : DMC_MAXTRQ;
    float accelFactor = pow(throttle / 100.0f, 1.5f);
    return reverse ? (accelFactor * maxTorque) : (-accelFactor * maxTorque);
}

void DMCHandler::scaleAndPackValues() {
    int DMC_TrqSlewrate_Scale = DMC_TrqSlewrate * 100;
    int DMC_MechPwrMaxMot_Scale = DMC_MechPwrMaxMot / 4;
    
    controllBuffer2DMC[0] = DMC_TrqSlewrate_Scale >> 8;
    controllBuffer2DMC[1] = DMC_TrqSlewrate_Scale & 0xFF;
    controllBuffer2DMC[2] = DMC_SpdSlewrate >> 8;
    controllBuffer2DMC[3] = DMC_SpdSlewrate & 0xFF;
    controllBuffer2DMC[4] = DMC_MechPwrMaxMot_Scale >> 8;
    controllBuffer2DMC[5] = DMC_MechPwrMaxMot_Scale & 0xFF;
}

void DMCHandler::calculateSpeed() {
    uint16_t ratio = LowRange ? REDUCED_RATIO : NORMAL_RATIO;
    float speed = DMC_SpdAct * 60.0f / ratio / DIFF_RATIO * WHEEL_CIRC;
    this->speed = static_cast<uint16_t>(speed);
}