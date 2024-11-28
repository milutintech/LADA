#include "system_control.h"

SystemControl::SystemControl() {}

void SystemControl::armBattery(bool arm) {
    if (arm && !HasPrecharged) {
        if (modeBSC != BSC6_BOOST) {
            modeBSC = BSC6_BOOST;
            lastModeChangeTime = millis();
            enableBSC = 0;
        }

        if (millis() - lastModeChangeTime >= 2000) {
            Hvoltage = BMS_U_BAT;
            enableBSC = 1;
        }

        if ((BSC6_HVVOL_ACT >= (BMS_U_BAT - 20)) && 
            (BSC6_HVVOL_ACT <= (BMS_U_BAT + 20)) && 
            (BSC6_HVVOL_ACT > 50)) {
            HasPrecharged = true;
            digitalWrite(CONTACTOR, HIGH);
            enableBSC = 0;
        }
    } else {
        batteryArmed = false;
        enableBSC = 0;
        HasPrecharged = false;
        digitalWrite(CONTACTOR, LOW);
    }
}

void SystemControl::armCoolingSys(bool arm) {
    if (batteryArmed && arm && 
        (DMC_TempInv > TEMP_INV_HIGH || 
         DMC_TempMot > TEMP_MOT_HIGH || 
         NLG_CoolingRequest > 50)) {
        digitalWrite(PUMP, HIGH);
    } else if (DMC_TempInv < TEMP_INV_LOW && 
               DMC_TempMot < TEMP_MOT_LOW && 
               NLG_CoolingRequest < 0) {
        digitalWrite(PUMP, LOW);
    }
}

void SystemControl::updateGearState() {
    // Read gear selector inputs
    int forwardValue = analogRead(GASPEDAL1);
    int reverseValue = analogRead(GASPEDAL2);
    
    bool isForwardHigh = forwardValue > 200;
    bool isReverseHigh = reverseValue > 200;
    
    const float RPM_THRESHOLD = 100.0f;
    
    if (DMC_SpdAct < RPM_THRESHOLD) {
        if (isForwardHigh && !isReverseHigh) {
            currentGear = Gear::Drive;
            shiftAttempted = false;
        } else if (!isForwardHigh && isReverseHigh) {
            currentGear = Gear::Reverse;
            shiftAttempted = false;
        } else {
            currentGear = Gear::Neutral;
            shiftAttempted = false;
        }
    } else if ((isForwardHigh && currentGear == Gear::Reverse) || 
               (isReverseHigh && currentGear == Gear::Drive)) {
        currentGear = Gear::Neutral;
        shiftAttempted = true;
    }
}