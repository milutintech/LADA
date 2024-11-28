// dmc_handler.cpp
#include "dmc_handler.h"
#include <algorithm>

DMCHandler::DMCHandler(mcp2515_can& can) : BaseCANHandler(can) {}

void DMCHandler::sendDMC() {
    DMC_TrqRq_Scale = calculateTorque();
    packControlMessage();
    packLimitMessage();
    scaleAndPackValues();
    
    sendMessage(DMCCTRL, 8, controllBufferDMC);
    sendMessage(DMCLIM, 8, limitBufferDMC);
    sendMessage(DMCCTRL2, 8, controllBuffer2DMC);
}

void DMCHandler::receiveDMC() {
    CANMessage msg;
    if (!receiveMessage(msg)) return;
    
    switch(msg.id) {
        case 0x258: processStatusMessage(msg); break;
        case 0x458: processTemperatureMessage(msg); break;
        case 0x259: processVoltageCurrentMessage(msg); break;
    }
}

void DMCHandler::processStatusMessage(const CANMessage& msg) {
    DMC_Ready = msg.data[0] & 0x80;
    DMC_Running = msg.data[0] & 0x40;
    DMC_SpdAct = static_cast<int16_t>((msg.data[6] << 8) | msg.data[7]);
}

void DMCHandler::processTemperatureMessage(const CANMessage& msg) {
    DMC_TempInv = ((msg.data[0] << 8) | msg.data[1]) * 0.5f;
    DMC_TempMot = ((msg.data[2] << 8) | msg.data[3]) * 0.5f;
}

void DMCHandler::processVoltageCurrentMessage(const CANMessage& msg) {
    DMC_DcVltAct = ((msg.data[0] << 8) | msg.data[1]) * 0.1f;
    DMC_DcCurrAct = ((msg.data[2] << 8) | msg.data[3]) * 0.1f;
    DMC_MechPwr = ((msg.data[6] << 8) | msg.data[7]) * 16;
}

float DMCHandler::calculateRegenTorque(float speed, float throttle) {
    using namespace VehicleParams;
    
    if (abs(speed) < Regen::ZERO_SPEED) return 0;
    
    if (speed < -Regen::MIN_SPEED) {
        if (throttle < Regen::END_POINT) {
            float regenFactor = 1.0f - (throttle / Regen::END_POINT);
            regenFactor = pow(regenFactor, 1.8f);
            float speedFactor = (abs(speed) > 1000.0f) ? 1.2f : 1.0f;
            return regenFactor * (Motor::MAX_REQ_TRQ * 0.7f) * speedFactor;
        }
        if (throttle < Regen::COAST_END) return 0;
    }
    
    return calculateDriveTorque(throttle, false);
}

float DMCHandler::calculateDriveTorque(float throttle, bool reverse) {
    using namespace VehicleParams::Motor;
    
    if (throttle <= 5.0f) return 0;
    
    float maxTorque = reverse ? MAX_REVERSE_TRQ : MAX_TRQ;
    float accelFactor = pow(throttle / 100.0f, 1.5f);
    return reverse ? (accelFactor * maxTorque) : (-accelFactor * maxTorque);
}

void DMCHandler::calculateSpeed() {
    using namespace VehicleParams::Transmission;
    float ratio = LowRange ? REDUCED_RATIO : NORMAL_RATIO;
    float speed = DMC_SpdAct * 60.0f / ratio / DIFF_RATIO * WHEEL_CIRC;
    DMC_SpdAct = speed;
}

float DMCHandler::calculateMedianThrottle() {
    std::sort(std::begin(sampleSetPedal), std::end(sampleSetPedal));
    return sampleSetPedal[2];
}

void DMCHandler::packControlMessage() {
    uint8_t controlByte = (enableDMC << 7) | (modeDMC << 6) | 
                         (1 << 5) | (1 << 1) | 1;
    
    controllBufferDMC[0] = controlByte;
    controllBufferDMC[2] = DMC_SpdRq >> 8;
    controllBufferDMC[3] = DMC_SpdRq & 0xFF;
    controllBufferDMC[4] = (DMC_TrqRq_Scale * 10) >> 8;
    controllBufferDMC[5] = (DMC_TrqRq_Scale * 10) & 0xFF;
}

void DMCHandler::packLimitMessage() {
    limitBufferDMC[0] = DMC_DcVLimMot >> 8;
    limitBufferDMC[1] = DMC_DcVLimMot & 0xFF;
    limitBufferDMC[2] = DMC_DcVLimGen >> 8;
    limitBufferDMC[3] = DMC_DcVLimGen & 0xFF;
}

void DMCHandler::scaleAndPackValues() {
    int DMC_TrqSlewrate_Scale = VehicleParams::Motor::MAX_ACCEL_STEP * 100;
    int DMC_MechPwrMaxMot_Scale = VehicleParams::Power::MAX_DMC_CURRENT / 4;
    
    controllBuffer2DMC[0] = DMC_TrqSlewrate_Scale >> 8;
    controllBuffer2DMC[1] = DMC_TrqSlewrate_Scale & 0xFF;
    controllBuffer2DMC[2] = DMC_SpdRq >> 8;
    controllBuffer2DMC[3] = DMC_SpdRq & 0xFF;
    controllBuffer2DMC[4] = DMC_MechPwrMaxMot_Scale >> 8;
    controllBuffer2DMC[5] = DMC_MechPwrMaxMot_Scale & 0xFF;
}

int16_t DMCHandler::calculateTorque() {
    float throttle = calculateMedianThrottle();
    float torque;
    
    if (DMC_SpdAct < 0) {
        torque = calculateRegenTorque(DMC_SpdAct, throttle);
    } else {
        torque = calculateDriveTorque(throttle, false);
    }
    
    applyTorqueLimits(torque, false);
    return static_cast<int16_t>(torque);
}