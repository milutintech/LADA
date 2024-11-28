#include "vehicle_control.h"
#include "config.h"
#include "ADS1X15.h"

VehicleControl::VehicleControl(ADS1115& ads) 
    : ads(ads)
    , currentDrivingMode(DriveMode::REGEN)
    , isOPDEnabled(false)
    , isRegenEnabled(true)
    , lastTorque(0)
    , motorSpeed(0)
    , wasInDeadband(false)
    , wasEnabled(false)
{
}

int16_t VehicleControl::calculateTorque() {
    // Sample pedal position
    int32_t sampledPotiValue = samplePedalPosition();
    
    // Map raw pedal value to 0-100%
    float rawThrottle = map(sampledPotiValue, ADC::MinValPot, ADC::MaxValPot, 0, 100);
    rawThrottle = constrain(rawThrottle, 0.0f, 100.0f);
    
    // Quick exit for legacy mode with released pedal
    if (!isOPDEnabled && !isRegenEnabled && rawThrottle < 1.0f) {
        lastTorque = 0;
        enableDMC = false;
        return 0;
    }
    
    // Calculate throttle position with gamma correction
    float throttlePosition = pow(rawThrottle / 100.0f, 1.5f) * 100.0f;
    
    // Handle neutral gear
    if (currentGear == GearState::NEUTRAL) {
        lastTorque = 0;
        return 0;
    }

    // Calculate vehicle speed
    float speed = calculateVehicleSpeed();
    
    int16_t calculatedTorque = 0;
    
    switch(currentDrivingMode) {
        case DriveMode::LEGACY:
            calculatedTorque = handleLegacyMode(throttlePosition);
            break;
            
        case DriveMode::REGEN:
            calculatedTorque = handleRegenMode(throttlePosition, speed);
            break;
            
        case DriveMode::OPD:
            calculatedTorque = handleOPDMode(throttlePosition, speed);
            break;
    }
    
    // Apply torque rate limiting
    calculatedTorque = applyTorqueLimits(calculatedTorque);
    
    // Apply deadband with hysteresis
    calculatedTorque = applyDeadbandHysteresis(calculatedTorque);
    
    return calculatedTorque;
}

int32_t VehicleControl::samplePedalPosition() {
    int32_t total = 0;
    for (int i = 0; i < 4; i++) {
        total += ads.readADC(ADC::GASPEDAL1);
    }
    return total / 4;
}

float VehicleControl::calculateVehicleSpeed() {
    if(currentGearRatio == GearRatio::REDUCED) {
        return motorSpeed * 60 / VehicleParams::Transmission::REDUCED_RATIO 
               / VehicleParams::Transmission::DIFF_RATIO 
               * VehicleParams::Transmission::WHEEL_CIRC;
    } else {
        return motorSpeed * 60 / VehicleParams::Transmission::NORMAL_RATIO 
               / VehicleParams::Transmission::DIFF_RATIO 
               * VehicleParams::Transmission::WHEEL_CIRC;
    }
}

int16_t VehicleControl::handleLegacyMode(float throttlePosition) {
    float normalizedThrottle = pow(throttlePosition / 100.0f, 1.5f);
    return normalizedThrottle * (currentGear == GearState::DRIVE ? 
           -VehicleParams::Motor::MAX_TRQ : VehicleParams::Motor::MAX_REVERSE_TRQ);
}

int16_t VehicleControl::handleRegenMode(float throttlePosition, float speed) {
    const float ZERO_SPEED_WINDOW = 0.5f;
    const float MIN_SPEED_FOR_REGEN = 100.0f;
    const float HIGH_SPEED_THRESHOLD = 1000.0f;
    const float REGEN_END_POINT = 35.0f;
    const float COAST_END_POINT = 40.0f;
    
    if (currentGear == GearState::DRIVE) {
        if (abs(motorSpeed) < ZERO_SPEED_WINDOW) {
            // At standstill
            if (throttlePosition > COAST_END_POINT) {
                float accelFactor = (throttlePosition - COAST_END_POINT) / (100.0f - COAST_END_POINT);
                accelFactor = pow(accelFactor, VehicleParams::Control::PEDAL_GAMMA);
                return -accelFactor * VehicleParams::Motor::MAX_TRQ;
            }
            return 0;
        }
        else if (motorSpeed < -MIN_SPEED_FOR_REGEN) {
            // Forward motion
            if (throttlePosition < REGEN_END_POINT) {
                // Regen zone
                float regenFactor = 1.0f - (throttlePosition / REGEN_END_POINT);
                regenFactor = pow(regenFactor, 1.8f);
                
                float speedFactor = 1.0f;
                if (abs(motorSpeed) > HIGH_SPEED_THRESHOLD) {
                    speedFactor = 1.2f;
                }
                
                return regenFactor * VehicleParams::Motor::MAX_REQ_TRQ * speedFactor;
            }
            else if (throttlePosition < COAST_END_POINT) {
                // Coast zone
                return 0;
            }
            else {
                // Acceleration zone
                float accelFactor = (throttlePosition - COAST_END_POINT) / (100.0f - COAST_END_POINT);
                accelFactor = pow(accelFactor, VehicleParams::Control::PEDAL_GAMMA);
                return -accelFactor * VehicleParams::Motor::MAX_TRQ;
            }
        }
        else {
            // Wrong direction protection
            if (throttlePosition > COAST_END_POINT) {
                return -VehicleParams::Motor::MAX_TRQ * 0.3f;
            }
            return 0;
        }
    }
    return 0;
}

int16_t VehicleControl::handleOPDMode(float throttlePosition, float speed) {
    const float speedPercent = constrain(abs(speed) / MAX_VEHICLE_SPEED, 0.0f, 1.0f);
    const float coastPosition = VehicleParams::Control::COAST_POSITION_MIN + 
                              (speedPercent * VehicleParams::Control::COAST_POSITION_MAX);
    const float normalizedPosition = (throttlePosition - coastPosition) / coastPosition;
    
    if (currentGear == GearState::DRIVE) {
        return calculateForwardOPDTorque(normalizedPosition, speedPercent);
    }
    else if (currentGear == GearState::REVERSE) {
        return calculateReverseOPDTorque(normalizedPosition, speedPercent);
    }
    
    return 0;
}

int16_t VehicleControl::calculateForwardOPDTorque(float normalizedPosition, float speedPercent) {
    const float ZERO_SPEED_THRESHOLD = 0.5f;
    
    if (abs(motorSpeed) < ZERO_SPEED_THRESHOLD) {
        if (normalizedPosition > 0) {
            float accelFactor = pow(normalizedPosition, VehicleParams::Control::PEDAL_GAMMA);
            return -accelFactor * VehicleParams::Motor::MAX_TRQ;
        }
        return 0;
    }
    
    if (normalizedPosition > 0) {
        float accelFactor = pow(normalizedPosition, VehicleParams::Control::PEDAL_GAMMA);
        float speedBasedTorque = VehicleParams::Motor::MAX_TRQ * 
                                (1.0f - (speedPercent * 0.3f));
        return -accelFactor * speedBasedTorque;
    }
    else {
        float regenFactor = -normalizedPosition;
        float speedFactor = calculateRegenSpeedFactor();
        return regenFactor * VehicleParams::Motor::MAX_REQ_TRQ * speedFactor;
    }
}

int16_t VehicleControl::calculateReverseOPDTorque(float normalizedPosition, float speedPercent) {
    const float ZERO_SPEED_THRESHOLD = 0.5f;
    
    if (abs(motorSpeed) < ZERO_SPEED_THRESHOLD) {
        if (normalizedPosition > VehicleParams::Control::MIN_PEDAL_THRESHOLD) {
            float accelFactor = pow(normalizedPosition / 100.0f, VehicleParams::Control::PEDAL_GAMMA);
            return accelFactor * VehicleParams::Motor::MAX_REVERSE_TRQ;
        }
        return 0;
    }
    
    if (motorSpeed >= 0.0f) {
        if (normalizedPosition > 0) {
            if (normalizedPosition > VehicleParams::Control::MIN_PEDAL_THRESHOLD) {
                float accelFactor = pow(normalizedPosition, VehicleParams::Control::PEDAL_GAMMA);
                float speedBasedTorque = VehicleParams::Motor::MAX_REVERSE_TRQ * 
                                       (1.0f - (speedPercent * 0.3f));
                return accelFactor * speedBasedTorque;
            }
            return 0;
        }
        else {
            float regenFactor = -normalizedPosition;
            float speedFactor = calculateRegenSpeedFactor();
            return -regenFactor * VehicleParams::Motor::MAX_REVERSE_TRQ * speedFactor;
        }
    }
    else {
        // Wrong direction protection
        return VehicleParams::Motor::MAX_REVERSE_TRQ * 0.3f;
    }
}

float VehicleControl::calculateRegenSpeedFactor() {
    if (abs(motorSpeed) < VehicleParams::Regen::FADE_START) {
        float speedFactor = abs(motorSpeed) / VehicleParams::Regen::FADE_START;
        return constrain(speedFactor, 0.0f, 1.0f);
    }
    return 1.0f;
}

int16_t VehicleControl::applyTorqueLimits(int16_t requestedTorque) {
    float torqueDiff = requestedTorque - lastTorque;
    
    if (abs(requestedTorque) > abs(lastTorque)) {
        if (torqueDiff > VehicleParams::Motor::MAX_ACCEL_STEP) {
            requestedTorque = lastTorque + VehicleParams::Motor::MAX_ACCEL_STEP;
        }
        else if (torqueDiff < -VehicleParams::Motor::MAX_ACCEL_STEP) {
            requestedTorque = lastTorque - VehicleParams::Motor::MAX_ACCEL_STEP;
        }
    }
    else {
        if (torqueDiff > VehicleParams::Motor::MAX_DECEL_STEP) {
            requestedTorque = lastTorque + VehicleParams::Motor::MAX_DECEL_STEP;
        }
        else if (torqueDiff < -VehicleParams::Motor::MAX_DECEL_STEP) {
            requestedTorque = lastTorque - VehicleParams::Motor::MAX_DECEL_STEP;
        }
    }
    
    lastTorque = requestedTorque;
    return requestedTorque;
}

int16_t VehicleControl::applyDeadbandHysteresis(int16_t torque) {
    if (wasInDeadband) {
        if (abs(torque) > VehicleParams::Motor::TORQUE_DEADBAND_HIGH) {
            wasInDeadband = false;
            enableDMC = true;
        }
        else {
            torque = 0;
            enableDMC = true;
        }
    }
    else {
        if (abs(torque) < VehicleParams::Motor::TORQUE_DEADBAND_LOW) {
            wasInDeadband = true;
            torque = 0;
            enableDMC = true;
        }
        else {
            enableDMC = true;
        }
    }
    
    return torque;
}

void VehicleControl::setMotorSpeed(float speed) {
    motorSpeed = speed;
}

void VehicleControl::setCurrentGear(GearState gear) {
    currentGear = gear;
}

void VehicleControl::setDrivingMode(DriveMode mode) {
    currentDrivingMode = mode;
}

bool VehicleControl::isDMCEnabled() const {
    return enableDMC;
}
