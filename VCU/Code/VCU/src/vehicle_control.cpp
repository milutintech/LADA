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
    float throttlePosition = pow(rawThrottle / 100.0f, VehicleParams::Control::PEDAL_GAMMA) * 100.0f;
    
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
    float ratio = (currentGearRatio == GearRatio::REDUCED) ? 
                  VehicleParams::Transmission::REDUCED_RATIO : 
                  VehicleParams::Transmission::NORMAL_RATIO;
                  
    return motorSpeed * 60.0f / ratio / 
           VehicleParams::Transmission::DIFF_RATIO * 
           VehicleParams::Transmission::WHEEL_CIRC;
}

int16_t VehicleControl::handleLegacyMode(float throttlePosition) {
    float normalizedThrottle = pow(throttlePosition / 100.0f, VehicleParams::Control::PEDAL_GAMMA);
    return normalizedThrottle * (currentGear == GearState::DRIVE ? 
           -VehicleParams::Motor::MAX_TRQ : VehicleParams::Motor::MAX_REVERSE_TRQ);
}

int16_t VehicleControl::handleRegenMode(float throttlePosition, float speed) {
    if (currentGear == GearState::DRIVE) {
        if (abs(motorSpeed) < VehicleParams::Regen::ZERO_SPEED) {
            // At standstill
            if (throttlePosition > VehicleParams::Regen::COAST_END) {
                float accelFactor = (throttlePosition - VehicleParams::Regen::COAST_END) / 
                                  (100.0f - VehicleParams::Regen::COAST_END);
                return -accelFactor * VehicleParams::Motor::MAX_TRQ;
            }
            return 0;
        }
        else if (motorSpeed < -VehicleParams::Regen::MIN_SPEED) {
            // Forward motion
            if (throttlePosition < VehicleParams::Regen::END_POINT) {
                // Regen zone
                float regenFactor = 1.0f - (throttlePosition / VehicleParams::Regen::END_POINT);
                return regenFactor * VehicleParams::Motor::MAX_REQ_TRQ;
            }
            else if (throttlePosition < VehicleParams::Regen::COAST_END) {
                // Coast zone
                return 0;
            }
            else {
                // Acceleration zone
                float accelFactor = (throttlePosition - VehicleParams::Regen::COAST_END) / 
                                  (100.0f - VehicleParams::Regen::COAST_END);
                return -accelFactor * VehicleParams::Motor::MAX_TRQ;
            }
        }
    }
    return 0;
}

int16_t VehicleControl::handleOPDMode(float throttlePosition, float speed) {
    // Speed limit enforcement
    if (speed >= VehicleParams::OPD::MAX_SPEED) {
        // Only allow regen/negative torque at max speed
        float coastUpper = VehicleParams::OPD::PHI;
        if (throttlePosition <= coastUpper) {
            float normalizedPosition = throttlePosition / coastUpper;
            float regenTorque = VehicleParams::OPD::MAX_REGEN * 
                               (VehicleParams::Motor::MAX_TRQ / 100.0f) * 
                               (1.0f - normalizedPosition);
            digitalWrite(Pins::BCKLIGHT, regenTorque > VehicleParams::OPD::BRAKE_LIGHT_THRESHOLD ? HIGH : LOW);
            return currentGear == GearState::DRIVE ? regenTorque : -regenTorque;
        }
        return 0;
    }

    const float speedPercent = constrain(abs(speed) / VehicleParams::OPD::MAX_SPEED, 0.0f, 1.0f);
    
    // Calculate coast boundaries
    float coastUpper = VehicleParams::OPD::PHI * pow(speedPercent, 1.0f/VehicleParams::OPD::SHAPE_FACTOR);
    float coastLower = coastUpper - VehicleParams::OPD::COAST_RANGE * speedPercent;
    
    // Anti-rollback protection
    if (speed < VehicleParams::OPD::ROLLBACK_SPEED && throttlePosition < coastLower) {
        float rollbackTorque = VehicleParams::OPD::ROLLBACK_TORQUE * 
                              (VehicleParams::Motor::MAX_TRQ / 100.0f);
        return currentGear == GearState::DRIVE ? -rollbackTorque : rollbackTorque;
    }
    
    // Coast zone handling
    if (throttlePosition >= coastLower && throttlePosition <= coastUpper) {
        return 0;
    }
    
    if (throttlePosition > coastUpper) {
        // Acceleration with speed-based limiting
        float normalizedPosition = (throttlePosition - coastUpper) / (100.0f - coastUpper);
        float maxTorque = VehicleParams::Motor::MAX_TRQ;
        
        // Progressively reduce torque as we approach max speed
        float speedFactor = 1.0f - pow(speedPercent, 2.0f);
        
        if (speed < 20.0f) {
            maxTorque = maxTorque * (0.8f + (20.0f - speed) / 100.0f);
        }
        
        float torque = maxTorque * speedFactor * pow(normalizedPosition, 1.5f);
        return currentGear == GearState::DRIVE ? -torque : torque;
    } else {
        // Regenerative braking
        float normalizedPosition = throttlePosition / coastLower;
        float maxRegen = VehicleParams::OPD::MAX_REGEN * 
                        (VehicleParams::Motor::MAX_TRQ / 100.0f);
        
        if (speed > 60.0f) {
            maxRegen *= (1.0f - (speed - 60.0f) * 0.005f);
        }
        
        float regenTorque = maxRegen * (1.0f - normalizedPosition);
        digitalWrite(Pins::BCKLIGHT, regenTorque > VehicleParams::OPD::BRAKE_LIGHT_THRESHOLD ? HIGH : LOW);
        
        return currentGear == GearState::DRIVE ? regenTorque : -regenTorque;
    }
}

int16_t VehicleControl::applyTorqueLimits(int16_t requestedTorque) {
    float torqueDiff = requestedTorque - lastTorque;
    
    if (abs(requestedTorque) > abs(lastTorque)) {
        torqueDiff = constrain(torqueDiff, 
                             -VehicleParams::Motor::MAX_DECEL_STEP,
                             VehicleParams::Motor::MAX_ACCEL_STEP);
    } else {
        torqueDiff = constrain(torqueDiff, 
                             -VehicleParams::Motor::MAX_DECEL_STEP,
                             VehicleParams::Motor::MAX_DECEL_STEP);
    }
    
    lastTorque = lastTorque + torqueDiff;
    return lastTorque;
}

int16_t VehicleControl::applyDeadbandHysteresis(int16_t torque) {
    if (wasInDeadband) {
        if (abs(torque) > VehicleParams::Motor::TORQUE_DEADBAND_HIGH) {
            wasInDeadband = false;
            enableDMC = true;
        } else {
            torque = 0;
            enableDMC = true;
        }
    } else {
        if (abs(torque) < VehicleParams::Motor::TORQUE_DEADBAND_LOW) {
            wasInDeadband = true;
            torque = 0;
            enableDMC = true;
        } else {
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