#pragma once
#include "dmc_handler.h"

DMCHandler::DMCHandler(mcp2515_can& can) : canBus(can) {}

int16_t DMCHandler::calculateTorque() {
    int32_t sampledPotiValue = 0;
    int16_t torqueCalc = 0;
    
    // Sample throttle position
    for (int i = 0; i < 4; i++) {
        sampledPotiValue += sampleSetPedal[i];
    }
    sampledPotiValue /= 4;

    float rawThrottle = map(sampledPotiValue, MinValPot, MaxValPot, 0, 100);
    rawThrottle = constrain(rawThrottle, 0.0f, 100.0f);
    
    // Quick exit for neutral
    if (currentGear == Gear::Neutral) {
        lastTorque = 0;
        return 0;
    }

    updateSpeedCalculation();
    
    float throttlePosition = pow(rawThrottle / 100.0f, 1.5f) * 100.0f;
    
    // Calculate torque based on mode
    if (currentDrivingMode == DrivingMode::REGEN) {
        torqueCalc = calculateRegenTorque(DMC_SpdAct, throttlePosition);
    } else {
        torqueCalc = calculateDriveTorque(throttlePosition, currentGear == Gear::Reverse);
    }
    
    // Apply rate limiting
    float torqueDiff = torqueCalc - lastTorque;
    const float maxAccelStep = 8.0f;
    const float maxDecelStep = 25.0f;
    
    if (abs(torqueCalc) > abs(lastTorque)) {
        torqueCalc = lastTorque + constrain(torqueDiff, -maxAccelStep, maxAccelStep);
    } else {
        torqueCalc = lastTorque + constrain(torqueDiff, -maxDecelStep, maxDecelStep);
    }
    
    lastTorque = torqueCalc;
    
    // Apply deadband with hysteresis
    static bool wasInDeadband = false;
    if (wasInDeadband) {
        if (abs(torqueCalc) > 25) {
            wasInDeadband = false;
            enableDMC = true;
        } else {
            torqueCalc = 0;
            enableDMC = true;
        }
    } else {
        if (abs(torqueCalc) < 18) {
            wasInDeadband = true;
            torqueCalc = 0;
        }
        enableDMC = true;
    }
    
    return torqueCalc;
}

float DMCHandler::calculateRegenTorque(float rawSpeed, float throttlePosition) {
    const float ZERO_SPEED_WINDOW = 0.5f;
    const float MIN_SPEED_FOR_REGEN = 100.0f;
    const float REGEN_END_POINT = 35.0f;
    const float COAST_END_POINT = 40.0f;
    const float tau_rm = DMC_MAXREQTRQ * 0.7f;
    
    if (abs(rawSpeed) < ZERO_SPEED_WINDOW) {
        return 0;
    }
    
    if (rawSpeed < -MIN_SPEED_FOR_REGEN) {
        if (throttlePosition < REGEN_END_POINT) {
            float regenFactor = 1.0f - (throttlePosition / REGEN_END_POINT);
            regenFactor = pow(regenFactor, 1.8f);
            float speedFactor = (abs(rawSpeed) > 1000.0f) ? 1.2f : 1.0f;
            return regenFactor * tau_rm * speedFactor;
        } else if (throttlePosition < COAST_END_POINT) {
            return 0;
        }
    }
    
    return calculateDriveTorque(throttlePosition, false);
}

float DMCHandler::calculateDriveTorque(float throttlePosition, bool isReverse) {
    const float gamma = 1.5f;
    const float tau_am = isReverse ? MAX_REVERSE_TRQ : DMC_MAXTRQ;
    
    if (throttlePosition <= 5.0f) return 0;
    
    float accelFactor = pow(throttlePosition / 100.0f, gamma);
    return isReverse ? (accelFactor * tau_am) : (-accelFactor * tau_am);
}

void DMCHandler::sendDMC() {
    // Scale values
    DMC_TrqRq_Scale = calculateTorque();
    
    // Pack control message
    uint8_t controlByte = (enableDMC << 7) | (modeDMC << 6) | 
                         (oscLim << 5) | (negTrqSpd << 1) | posTrqSpd;
    
    controllBufferDMC[0] = controlByte;
    controllBufferDMC[2] = DMC_SpdRq >> 8;
    controllBufferDMC[3] = DMC_SpdRq & 0xFF;
    controllBufferDMC[4] = (DMC_TrqRq_Scale * 10) >> 8;
    controllBufferDMC[5] = (DMC_TrqRq_Scale * 10) & 0xFF;
    
    canBus.sendMsgBuf(DMCCTRL, 0, 8, controllBufferDMC);
}

void DMCHandler::receiveDMC() {
    if (canBus.checkReceive() != CAN_MSGAVAIL) return;
    
    uint8_t len;
    canBus.readMsgBuf(&len, readDataDMC);
    uint32_t id = canBus.getCanId();
    
    switch(id) {
        case 0x258:
            DMC_SpdAct = ((readDataDMC[6] << 8) | readDataDMC[7]);
            break;
            
        case 0x458:
            DMC_TempInv = ((readDataDMC[0] << 8) | readDataDMC[1]) * 0.5f;
            DMC_TempMot = ((readDataDMC[2] << 8) | readDataDMC[3]) * 0.5f;
            break;
    }
}

void DMCHandler::updateSpeedCalculation() {
    speed = DMC_SpdAct * 60 / 
            (LowRange ? REDUCED_RATIO : NORMAL_RATIO) / 
            DIFF_RATIO * WHEEL_CIRC;
}