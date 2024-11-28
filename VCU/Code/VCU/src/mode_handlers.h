#pragma once
#include "config.h"
#include "vehicle_state.h"

class ModeHandler {
public:
    virtual void enter() = 0;
    virtual void handle() = 0;
    virtual void handleCommunication() = 0;
    virtual void exit() = 0;
    virtual ~ModeHandler() = default;
};

class StandbyModeHandler : public ModeHandler {
public:
    void enter() override {
        digitalWrite(DMCKL15, LOW);
        digitalWrite(BSCKL15, LOW);
        digitalWrite(NLGKL15, LOW);
    }

    void handle() override {
        sysControl.armBattery(false);
        sysControl.armCoolingSys(false);
        
        if(digitalRead(NLG_HW_Wakeup)) {
            changeMode(VehicleMode::Charging);
        } else if(digitalRead(IGNITION)) {
            changeMode(VehicleMode::Run);
        } else if(!digitalRead(IGNITION) && !digitalRead(NLG_HW_Wakeup)) {
            esp_deep_sleep_start();
        }
    }

    void handleCommunication() override {
        // Minimal communication in standby
        bscHandler.receiveBSC();
    }

    void exit() override {
        // Cleanup before leaving standby
    }
};

class RunModeHandler : public ModeHandler {
public:
    void enter() override {
        digitalWrite(DMCKL15, HIGH);
        digitalWrite(BSCKL15, HIGH);
        sysControl.armBattery(true);
    }

    void handle() override {
        if (!digitalRead(IGNITION)) {
            changeMode(VehicleMode::Standby);
            return;
        }

        sysControl.armCoolingSys(true);
        sysControl.updateGearState();
        
        if (sysControl.isPreCharged()) {
            dmcHandler.processTorque();
        }

        digitalWrite(BCKLIGHT, sysControl.getCurrentGear() == Gear::Reverse ? HIGH : LOW);
    }

    void handleCommunication() override {
        static unsigned long lastFastCycle = 0;
        static unsigned long lastSlowCycle = 0;
        unsigned long currentTime = millis();

        if (currentTime - lastFastCycle >= 10) {
            dmcHandler.sendDMC();
            lastFastCycle = currentTime;
        }

        if (currentTime - lastSlowCycle >= 50) {
            bscHandler.sendBSC();
            sysControl.updateGearState();
            lastSlowCycle = currentTime;
        }

        dmcHandler.receiveDMC();
        bscHandler.receiveBSC();
    }

    void exit() override {
        sysControl.armBattery(false);
        sysControl.armCoolingSys(false);
    }
};

class ChargingModeHandler : public ModeHandler {
public:
    void enter() override {
        digitalWrite(NLGKL15, HIGH);
        sysControl.armCoolingSys(true);
    }

    void handle() override {
        if (conUlockInterrupt) {
            handleConnectorUnlock();
            return;
        }
        
        nlgHandler.chargeManage();
        sysControl.armBattery(true);
    }

    void handleCommunication() override {
        static unsigned long lastCycle = 0;
        unsigned long currentTime = millis();

        if (currentTime - lastCycle >= 10) {
            nlgHandler.sendNLG();
            lastCycle = currentTime;
        }

        nlgHandler.receiveNLG();
        bscHandler.receiveBSC();
    }

    void exit() override {
        sysControl.armBattery(false);
        nlgHandler.resetCharging();
    }

private:
    void handleConnectorUnlock() {
        if(NLG_S_ConLocked) {
            NLG_C_UnlockConRq = 1;
            NLG_Charged = 1;
        } else {
            conUlockInterrupt = 0;
            NLG_StateDem = NLG_DEM_STANDBY;
            changeMode(VehicleMode::Standby);
        }
    }
};