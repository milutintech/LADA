int16_t calculateTorque5S() {
    int32_t SampledPotiValue = 0;
    int16_t DMC_TorqueCalc = 0;
    static float lastTorque = 0;
    
    Serial.println("--- Debug Info ---");
    
    // Throttle Sampling and Averaging
    for (int i = 0; i < 4; i++) {
        SampledPotiValue += sampleSetPedal[i];
    }
    SampledPotiValue /= 4;

    // Map throttle position with progressive curve
    float rawThrottle = map(SampledPotiValue, MinValPot, MaxValPot, 0, 100);
    rawThrottle = constrain(rawThrottle, 0.0f, 100.0f);
    
    // Quick exit if pedal is fully released
    if (rawThrottle < 1.0f) {
        lastTorque = 0;
        enableDMC = 0;
        return 0;
    }
    
    // Progressive curve only for positive throttle values
    float throttlePosition = pow(rawThrottle / 100.0f, 2.0f) * 100.0f;
    
    Serial.print("Raw Throttle (%): "); Serial.println(rawThrottle);
    Serial.print("Smoothed Throttle (%): "); Serial.println(throttlePosition);

    // Handle Neutral gear
    if (currentGear == Neutral) {
        lastTorque = 0;
        return 0;
    }

    // Speed calculations
    float adjustedSpeed = currentGear == Drive ? -DMC_SpdAct : DMC_SpdAct;
    
    if (LowRange) {
        speed = adjustedSpeed * 60 / REDUCED_RATIO / DIFF_RATIO * WHEEL_CIRC / 1000;
    } else {
        speed = adjustedSpeed * 60 / NORMAL_RATIO / DIFF_RATIO * WHEEL_CIRC / 1000;
    }

    if (isOPDEnabled) {
        // ... OPD logic remains the same as before ...
    } else {
        // Legacy Mode with quicker response
        float normalizedThrottle = pow(rawThrottle / 100.0f, 1.8f);  // Slightly less aggressive curve
        DMC_TorqueCalc = normalizedThrottle * (currentGear == Drive ? DMC_MAXTRQ : MAX_REVERSE_TRQ);
        Serial.println("Mode: Legacy (OPD Disabled)");
    }

    // Apply direction-specific inversion
    DMC_TorqueCalc = (currentGear == Drive) ? -DMC_TorqueCalc : DMC_TorqueCalc;

    // Asymmetric torque rate limiting
    float torqueDiff = DMC_TorqueCalc - lastTorque;
    
    // Different rate limits for acceleration and deceleration
    float maxAccelStep = 8.0f;    // Limit rate of increasing torque
    float maxDecelStep = 25.0f;   // Allow faster decrease in torque
    
    // Apply rate limiting based on whether we're increasing or decreasing torque
    if (abs(DMC_TorqueCalc) > abs(lastTorque)) {
        // Accelerating - apply stricter limit
        if (torqueDiff > maxAccelStep) {
            DMC_TorqueCalc = lastTorque + maxAccelStep;
        } else if (torqueDiff < -maxAccelStep) {
            DMC_TorqueCalc = lastTorque - maxAccelStep;
        }
    } else {
        // Decelerating - allow faster changes
        if (torqueDiff > maxDecelStep) {
            DMC_TorqueCalc = lastTorque + maxDecelStep;
        } else if (torqueDiff < -maxDecelStep) {
            DMC_TorqueCalc = lastTorque - maxDecelStep;
        }
    }
    
    // Override rate limiting if throttle is very low
    if (rawThrottle < 5.0f && abs(DMC_TorqueCalc) > abs(lastTorque)) {
        DMC_TorqueCalc = 0;  // Quick cutoff at very low throttle
    }
    
    lastTorque = DMC_TorqueCalc;

    // Deadband with hysteresis
    static bool wasInDeadband = false;
    if (wasInDeadband) {
        if (abs(DMC_TorqueCalc) > 25) {
            wasInDeadband = false;
            enableDMC = 1;
        } else {
            DMC_TorqueCalc = 0;
            enableDMC = 0;
        }
    } else {
        if (abs(DMC_TorqueCalc) < 18) {
            wasInDeadband = true;
            DMC_TorqueCalc = 0;
            enableDMC = 0;
        } else {
            enableDMC = 1;
        }
    }

    Serial.print("Final Torque: "); Serial.println(DMC_TorqueCalc);
    Serial.print("DMC Enabled: "); Serial.println(enableDMC);
    Serial.println("---------------");
    
    return DMC_TorqueCalc;
}