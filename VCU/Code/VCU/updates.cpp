// REGEN MOD

#define REGEN_BRAKING_GAIN      2    // Gain factor for regenerative braking torque calculation
#define MAX_REGEN_TRQ           500  // Maximum regenerative braking torque (adjust as needed)
#define TORQUE_ENABLE_DEADBAND  5    // Deadband threshold for enabling/disabling DMC
#define DEAD_BAND              14    // Existing deadband threshold for torque

int16_t calculateTorque5S() {
    int32_t sampledPotiValue = 0;
    int16_t throttleTorque = 0;
    int16_t regenTorque = 0;
    int16_t totalTorque = 0;

    // Handle Neutral Gear
    if (currentGear == Neutral) {
        // In Neutral, no torque is required on DMC
        throttleTorque = 0;
        regenTorque = 0;
        totalTorque = 0;

        // Disable DMC
        enableDMC = 0;

        return totalTorque;
    }

    // Sum values in sampleSetPedal (assuming sampleSetPedal[0-3] contains valid data)
    for (int i = 0; i < 4; i++) {
        sampledPotiValue += sampleSetPedal[i];
    }

    // Calculate average
    sampledPotiValue /= 4;

    // Map and constrain torque based on current gear
    if (currentGear == Drive) {
        // Map potentiometer value to Drive torque range
        sampledPotiValue = map(sampledPotiValue, MinValPot, MaxValPot, 0, DMC_MAXTRQ);
        // Constrain torque to [0, DMC_MAXTRQ]
        throttleTorque = (int16_t)constrain(sampledPotiValue, 0, DMC_MAXTRQ);
    } 
    else if (currentGear == Reverse) {
        // Map potentiometer value to Reverse torque range
        sampledPotiValue = map(sampledPotiValue, MinValPot, MaxValPot, 0, MAX_REVERSE_TRQ);
        // Constrain torque to [0, MAX_REVERSE_TRQ] and apply negative sign for Reverse
        throttleTorque = -(int16_t)constrain(sampledPotiValue, 0, MAX_REVERSE_TRQ);
    }

    // Apply deadband of ±14 to throttle torque
    if (throttleTorque > -DEAD_BAND && throttleTorque < DEAD_BAND) {
        throttleTorque = 0;
    }

    // Calculate Regenerative Braking Torque
    if (currentGear == Drive && DMC_SpdAct > 0) {
        // Moving forward, apply negative torque for regenerative braking
        regenTorque = -constrain((int16_t)(REGEN_BRAKING_GAIN * DMC_SpdAct), -MAX_REGEN_TRQ, 0);
    }
    else if (currentGear == Reverse && DMC_SpdAct < 0) {
        // Moving reverse, apply positive torque for regenerative braking
        regenTorque = constrain((int16_t)(REGEN_BRAKING_GAIN * (-DMC_SpdAct)), 0, MAX_REGEN_TRQ);
    }
    else {
        regenTorque = 0;
    }

    // Apply regenerative braking deadband
    if (regenTorque > -DEAD_BAND && regenTorque < DEAD_BAND) {
        regenTorque = 0;
    }

    // Combine Throttle and Regenerative Torques
    if (throttleTorque == 0) {
        // Only regenerative braking is applied
        totalTorque = regenTorque;
    }
    else {
        // Regenerative braking is applied partially when throttle is pressed
        totalTorque = throttleTorque + (regenTorque / 2);  // Apply half of regen torque
    }

    // Ensure combined torque does not exceed system limits based on gear
    if (currentGear == Drive) {
        // In Drive, total torque should be within [-MAX_REGEN_TRQ, DMC_MAXTRQ]
        totalTorque = constrain(totalTorque, -MAX_REGEN_TRQ, DMC_MAXTRQ);
    }
    else if (currentGear == Reverse) {
        // In Reverse, total torque should be within [-MAX_REVERSE_TRQ, MAX_REGEN_TRQ]
        totalTorque = constrain(totalTorque, -MAX_REVERSE_TRQ, MAX_REGEN_TRQ);
    }

    // Apply deadband again after combining throttle and regen torques
    if (totalTorque > -DEAD_BAND && totalTorque < DEAD_BAND) {
        totalTorque = 0;
    }

    // Set enableDMC based on totalTorque within ±TORQUE_ENABLE_DEADBAND
    if (totalTorque > -TORQUE_ENABLE_DEADBAND && totalTorque < TORQUE_ENABLE_DEADBAND) {
        enableDMC = 0;  // Disable DMC
    } else {
        enableDMC = 1;  // Enable DMC
    }

    // Optional Debugging Output
    /*
    Serial.print("Throttle Torque: ");
    Serial.print(throttleTorque);
    Serial.print(" | Regen Torque: ");
    Serial.print(regenTorque);
    Serial.print(" | Total Torque: ");
    Serial.println(totalTorque);
    */

    return totalTorque;
}

//Limiting messages

void reciveINFO(){
  if (CAN_MSGAVAIL != CAN.checkReceive()) {
      return;
  }
  // read data, len: data length, buf: data buf
  CAN.readMsgBuf(&len, readDataBMS);
  id = CAN.getCanId();
  type = (CAN.isExtendedFrame() << 0) | (CAN.isRemoteRequest() << 1);
  
  if(id == 0x003){
    MAX_SOC = readDataDMC[0];
    MAX_TrqRq = (readDataINFO[1] | (readDataINFO[2] << 8));
    NLG_AcCurrLimMax = readDataINFO[3];
    OFFROAD_MODE = readDataINFO[5];
  }   
}
