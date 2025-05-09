#include "DemoMode.h"

DemoMode::DemoMode(DisplayController* disp, SpeedometerController* speedo)
    : display(disp), speedo(speedo) {
}

void DemoMode::loop() {
    unsigned long currentMillis = millis();
    
    // Update every 50ms
    if (currentMillis - lastUpdate >= 20) {
        lastUpdate = currentMillis;
        
        // Update kilometer counters
        kmDecimal++;
        if (kmDecimal >= 10) {
            kmDecimal = 0;
            totalKm++;
            tripKm++;
        }
        display->displayTotalKm(totalKm);
        display->displayTripKm(tripKm, kmDecimal);
        
        // Update torque (-100 to +400)
        if (increasing) {
            torque += 5;
            if (torque >= 400) increasing = false;
        } else {
            torque -= 5;
            if (torque <= -100) increasing = true;
        }
        Serial.printf("Setting torque to: %d\n", torque);
        speedo->updateTorque(torque);
        
        // Update SOC (0-100%)
        static unsigned long lastSocUpdate = 0;
        if (currentMillis - lastSocUpdate >= 500) {  // Every second
            lastSocUpdate = currentMillis;
            soc = (soc > 0) ? soc - 1 : 100;
            Serial.printf("Setting SOC to: %d%%\n", soc);
            speedo->updateSOC(soc);
        }
        
        // Update temperature (30-110°C)
        static unsigned long lastTempUpdate = 0;
        if (currentMillis - lastTempUpdate >= 250) {  // Every 0.5 seconds
            lastTempUpdate = currentMillis;
            temperature++;
            if (temperature > 110) temperature = 30;
            Serial.printf("Setting temperature to: %d°C\n", temperature);
            speedo->updateTemperature(temperature);
        }
        
        // Update error lights
        static unsigned long lastErrorUpdate = 0;
        if (currentMillis - lastErrorUpdate >= 1000) {  // Every 2 seconds
            lastErrorUpdate = currentMillis;
            errorFlags = (1 << currentError);
            Serial.printf("Setting error flag: 0x%04X (light %d)\n", errorFlags, currentError);
            speedo->updateErrorLights(errorFlags);
            currentError = (currentError + 1) % 13;
        }
        
        // Update speed (0-200)
        speed = (speed + 1) % 201;
        //display->displaySpeed(speed);
        display->displaySpeed(888);
        
        // Change drive mode
        if (currentMillis - lastModeChange >= modeChangeInterval) {
            lastModeChange = currentMillis;
            driveMode = static_cast<DisplayController::DriveMode>((static_cast<int>(driveMode) + 1) % 5);
            display->displayDriveMode(driveMode);
        }
        
        // Important: Show the updates!
        speedo->show();
    }
}