#pragma once
#include "mode_handlers.h"

class ModeManager {
public:
    ModeManager() : currentHandler(nullptr) {
        handlers[static_cast<int>(VehicleMode::Standby)] = new StandbyModeHandler();
        handlers[static_cast<int>(VehicleMode::Run)] = new RunModeHandler();
        handlers[static_cast<int>(VehicleMode::Charging)] = new ChargingModeHandler();
    }

    void changeMode(VehicleMode newMode) {
        if (currentHandler) {
            currentHandler->exit();
        }
        currentHandler = handlers[static_cast<int>(newMode)];
        currentHandler->enter();
        currentMode = newMode;
    }

    void handleCurrentMode() {
        if (currentHandler) {
            currentHandler->handle();
        }
    }

    void handleCommunication() {
        if (currentHandler) {
            currentHandler->handleCommunication();
        }
    }

    ~ModeManager() {
        for (auto handler : handlers) {
            delete handler;
        }
    }

private:
    ModeHandler* currentHandler;
    ModeHandler* handlers[3];
    VehicleMode currentMode;
};