#include "DemoMode.h"

DemoMode::DemoMode(DisplayController* disp, SpeedometerController* speedo)
    : display(disp), speedo(speedo) {
}

void DemoMode::loop() {
    unsigned long currentMillis = millis();
    
    // Update every 20ms (50Hz)
    if (currentMillis - lastUpdate >= 20) {
        lastUpdate = currentMillis;
        
        // Update kilometer counters - increment by 0.1 km every 0.5 seconds
        static unsigned long lastKmUpdate = 0;
        if (currentMillis - lastKmUpdate >= 500) {  // Update every 0.5 seconds
            lastKmUpdate = currentMillis;

            // Add 0.1 km to both total and trip odometers
            // This simulates driving at ~720 km/h (0.1 km per 0.5 seconds)
            // The addDistance() function handles updating both odometers AND the displays
            speedo->addDistance(0.1);

            delayMicroseconds(500);  // Delay before any Bus 2 operations
        }
        
        // Update DC current for torque bar (-450A to +450A)
        // Simulate driving & regen cycles
        if (increasing) {
            dcCurrent += 5;
            if (dcCurrent >= 450) increasing = false;
        } else {
            dcCurrent -= 5;
            if (dcCurrent <= -150) increasing = true;
        }
        speedo->updateTorque(dcCurrent);
        
        // Update SOC (0-100%)
        static unsigned long lastSocUpdate = 0;
        if (currentMillis - lastSocUpdate >= 500) {  // Every 0.5 second
            lastSocUpdate = currentMillis;
            soc = (soc > 0) ? soc - 1 : 100;
            speedo->updateSOC(soc);
        }
        
        // Update temperature (30-110°C)
        static unsigned long lastTempUpdate = 0;
        if (currentMillis - lastTempUpdate >= 250) {  // Every 0.25 seconds
            lastTempUpdate = currentMillis;
            temperature++;
            if (temperature > 110) temperature = 30;
            speedo->updateTemperature(temperature);
        }
        
        // Update error lights
        static unsigned long lastErrorUpdate = 0;
        if (currentMillis - lastErrorUpdate >= 1000) {  // Every second
            lastErrorUpdate = currentMillis;
            errorFlags = (1 << currentError);
            speedo->updateErrorLights(errorFlags);
            currentError = (currentError + 1) % 13;
        }
        
        // Update speed (0-200 km/h)
        static unsigned long lastSpeedUpdate = 0;
        // Offset by 50ms to avoid collision with distance updates (which happen at 0, 500, 1000ms...)
        if (currentMillis - lastSpeedUpdate >= 100) {  // Every 0.1 seconds
            lastSpeedUpdate = currentMillis;

            // Create a realistic speed curve
            if (speedIncreasing) {
                speed += random(1, 3);  // Random increase for realism
                if (speed >= 120) speedIncreasing = false;
            } else {
                speed -= random(1, 3);  // Random decrease
                if (speed <= 0) {
                    speed = 0;
                    speedIncreasing = true;
                }
            }

            display->displaySpeed(speed);
        }
        
        // Change drive mode
        if (currentMillis - lastModeChange >= modeChangeInterval) {
            lastModeChange = currentMillis;
            driveMode = static_cast<DisplayController::DriveMode>((static_cast<int>(driveMode) + 1) % 5);
            display->displayDriveMode(driveMode);
        }
        
        // Update brightness periodically
        static unsigned long lastBrightnessUpdate = 0;
        if (currentMillis - lastBrightnessUpdate >= 5000) {  // Every 5 seconds
            lastBrightnessUpdate = currentMillis;
            brightness = (brightness <= 100) ? 200 : 100;  // Toggle between two levels
            speedo->setIllumination(brightness);
        }
        
        // No need to call show here as it's handled in the main loop
    }
}