#include "state_manager.h"
#include "config.h"
#include <esp_sleep.h>

StateManager::StateManager()
    : currentState(VehicleState::STANDBY)
    , batteryArmed(false)
    , hasPreCharged(false)
    , nlgCharged(false)
    , connectorLocked(false)
    , unlockPersist(false)
    , conUlockInterrupt(false)
    , lastModeChangeTime(0)
    , lastPrechargeAttempt(0)
    , enableBSC(false)
    , modeBSC(false)
    , errorLatch(false)
    , chargerState(ChargerStates::NLG_ACT_SLEEP)
    , chargerStateDemand(ChargerStates::NLG_DEM_STANDBY)
    , chargeLedDemand(0)
{
    // Initialize GPIO outputs to safe states
    digitalWrite(Pins::DMCKL15, LOW);
    digitalWrite(Pins::BSCKL15, LOW);
    digitalWrite(Pins::NLGKL15, LOW);
    digitalWrite(Pins::CONTACTOR, LOW);
    digitalWrite(Pins::PUMP, LOW);
}

void StateManager::handleWakeup() {
    Serial.println("Handling wakeup...");
    uint8_t reason = getWakeupReason();
    Serial.print("Wakeup reason: ");
    Serial.println(reason);
    
    if (digitalRead(Pins::NLG_HW_Wakeup)) {
        Serial.println("NLG_HW_Wakeup pin is HIGH");
        transitionToCharging();
    } else if (digitalRead(Pins::IGNITION)) {
        Serial.println("IGNITION pin is HIGH");
        transitionToRun();
    } else {
        Serial.println("No wake signals, going to standby");
        transitionToStandby();
    }
}

void StateManager::update() {
    switch(currentState) {
        case VehicleState::STANDBY:
            handleStandbyState();
            break;
            
        case VehicleState::RUN:
            handleRunState();
            break;
            
        case VehicleState::CHARGING:
            handleChargingState();
            break;
    }
}

void StateManager::handleStandbyState() {
    // Disable all systems in standby
    digitalWrite(Pins::DMCKL15, LOW);
    digitalWrite(Pins::BSCKL15, LOW);
    digitalWrite(Pins::NLGKL15, LOW);
    
    batteryArmed = false;
    hasPreCharged = false;
    enableBSC = false;
    enableDMC = false;
    
    armBattery(false);
    armCoolingSys(false);
    
    // Check for state transitions
    if(digitalRead(Pins::NLG_HW_Wakeup)) {
        transitionToCharging();
    }
    if(digitalRead(Pins::IGNITION)) {
        transitionToRun();
    }
    
    // Enter deep sleep if no active inputs
    if((!digitalRead(Pins::IGNITION)) && (!digitalRead(Pins::NLG_HW_Wakeup))) {
        esp_deep_sleep_start();
    }
}

void StateManager::handleRunState() {
    if (!digitalRead(Pins::IGNITION)) {
        transitionToStandby();
        return;
    }
    
    armCoolingSys(true);
    armBattery(true);
    
    digitalWrite(Pins::DMCKL15, HIGH);
    digitalWrite(Pins::BSCKL15, HIGH);
    
    // Update reverse light based on gear state
    digitalWrite(Pins::BCKLIGHT, currentGear == GearState::REVERSE ? HIGH : LOW);
}

void StateManager::handleChargingState() {
    digitalWrite(Pins::NLGKL15, HIGH);
    digitalWrite(Pins::BSCKL15, HIGH);
    
    if(conUlockInterrupt) {
        handleConnectorUnlock();
    }
    
    armCoolingSys(true);
    chargeManage();
}

void StateManager::armBattery(bool arm) {
    if (arm) {
        if (!hasPreCharged) {
            // Start precharge process
            if (modeBSC != BSCModes::BSC6_BOOST) {
                modeBSC = BSCModes::BSC6_BOOST;
                lastModeChangeTime = millis();
                enableBSC = false;
            }

            if (millis() - lastModeChangeTime >= Constants::MODE_CHANGE_DELAY_MS) {
                hvVoltage = batteryVoltage;
                enableBSC = true;
                
                // Check if precharge is complete
                if ((hvVoltageActual >= (batteryVoltage - 20)) && 
                    (hvVoltageActual <= (batteryVoltage + 20)) && 
                    (hvVoltageActual > 50)) {
                    hasPreCharged = true;
                    digitalWrite(Pins::CONTACTOR, HIGH);
                    enableBSC = false;
                }
            }
        } else {
            batteryArmed = true;
            errorLatch = true;
            delay(100);
            errorLatch = false;

            if (modeBSC != BSCModes::BSC6_BUCK) {
                modeBSC = BSCModes::BSC6_BUCK;
                lastModeChangeTime = millis();
                enableBSC = false;
            }

            if (millis() - lastModeChangeTime >= Constants::MODE_CHANGE_DELAY_MS) {
                enableBSC = true;
            }
        }
    } else {
        batteryArmed = false;
        enableBSC = false;
        hasPreCharged = false;
        digitalWrite(Pins::CONTACTOR, LOW);
    }
}

void StateManager::armCoolingSys(bool arm) {
    if (batteryArmed) {
        if (arm && ((inverterTemp > VehicleParams::Temperature::INV_HIGH) || 
                   (motorTemp > VehicleParams::Temperature::MOT_HIGH) || 
                   (coolingRequest > 50))) {
            digitalWrite(Pins::PUMP, HIGH);
        } else if ((inverterTemp < VehicleParams::Temperature::INV_LOW) && 
                  (motorTemp < VehicleParams::Temperature::MOT_LOW) && 
                  (coolingRequest < 0)) {
            digitalWrite(Pins::PUMP, LOW);
        }
    }
}

void StateManager::chargeManage() {
    static unsigned long unlockTimeout = 0;

    if (currentState == VehicleState::CHARGING) {
        switch (chargerState) {
            case ChargerStates::NLG_ACT_SLEEP:
                chargeLedDemand = 0;
                chargerStateDemand = ChargerStates::NLG_DEM_STANDBY;
                break;

            case ChargerStates::NLG_ACT_STANDBY:
                chargeLedDemand = 1;
                if (nlgCharged) {
                    chargerStateDemand = ChargerStates::NLG_DEM_SLEEP;
                    transitionToStandby();
                    unlockConnectorRequest = true;
                    unlockPersist = true;
                    unlockTimeout = millis();
                }
                break;

            case ChargerStates::NLG_ACT_READY2CHARGE:
                chargeLedDemand = 3;
                if (unlockPersist || unlockConnectorRequest) {
                    chargerStateDemand = ChargerStates::NLG_DEM_STANDBY;
                } else if (hasPreCharged) {
                    chargerStateDemand = ChargerStates::NLG_DEM_CHARGE;
                }
                armBattery(true);
                break;

            case ChargerStates::NLG_ACT_CHARGE:
                chargeLedDemand = 4;
                armBattery(true);
                break;

            default:
                armBattery(false);
                break;
        }

        if (unlockPersist) {
            handlePersistentUnlock(unlockTimeout);
        }
    } else {
        armBattery(false);
        digitalWrite(Pins::NLGKL15, LOW);
    }
}

void StateManager::handlePersistentUnlock(unsigned long& unlockTimeout) {
    unlockConnectorRequest = true;
    if (!connectorLocked) {
        chargerStateDemand = ChargerStates::NLG_DEM_STANDBY;
        transitionToStandby();
        chargeLedDemand = 0;
    } else if (millis() - unlockTimeout > Constants::UNLOCK_TIMEOUT_MS) {
        unlockTimeout = millis();
    }
}

void StateManager::handleConnectorUnlock() {
    if (connectorLocked) {
        unlockConnectorRequest = true;
        nlgCharged = true;
    } else {
        conUlockInterrupt = false;
        chargerStateDemand = ChargerStates::NLG_DEM_STANDBY;
        transitionToStandby();
    }
}

uint8_t StateManager::getWakeupReason() {
    uint64_t GPIO_reason = esp_sleep_get_ext1_wakeup_status();
    return (log(GPIO_reason)) / log(2);
}

void StateManager::transitionToStandby() {
    Serial.println("Transitioning to STANDBY state");
    currentState = VehicleState::STANDBY;
    nlgCharged = false;
    enableBSC = false;
    enableDMC = false;
    armBattery(false);
    armCoolingSys(false);
    Serial.println("Now in STANDBY state");
}

void StateManager::transitionToRun() {
    Serial.println("Transitioning to RUN state");
    currentState = VehicleState::RUN;
    nlgCharged = false;
    Serial.println("Now in RUN state");
}

void StateManager::transitionToCharging() {
    Serial.println("Transitioning to CHARGING state");
    currentState = VehicleState::CHARGING;
    Serial.println("Now in CHARGING state");
}