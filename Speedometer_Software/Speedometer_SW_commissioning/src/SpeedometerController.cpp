#include "SpeedometerController.h"
#include "DisplayController.h"

extern DisplayController* display;

SpeedometerController::SpeedometerController() : 
    pixels(TOTAL_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800),
    can(CAN_CS_PIN) {}

void SpeedometerController::debugPrint(String msg) {
    #if DEBUG_PRINTS
    Serial.println(msg);
    #endif
}

bool SpeedometerController::begin() {
    debugPrint("Initializing SpeedometerController...");
    
    // Initialize NeoPixels
    pixels.begin();
    pixels.setBrightness(255);
    pixels.clear();
    pixels.show();
    delay(100);  // Give the NeoPixels time to initialize

    // Test pattern - light up each section in different colors
    debugPrint("Running startup test pattern...");
    
    // Torque section - Red
    for(int i = 0; i < TORQUE_PIXELS; i++) {
        pixels.setPixelColor(i, pixels.Color(255, 0, 0));
    }
    pixels.show();
    delay(500);

    // SOC section - Green
    for(int i = TORQUE_PIXELS; i < TORQUE_PIXELS + SOC_PIXELS; i++) {
        pixels.setPixelColor(i, pixels.Color(0, 255, 0));
    }
    pixels.show();
    delay(500);

    // SOC markers - Blue
    for(int i = TORQUE_PIXELS + SOC_PIXELS; i < TORQUE_PIXELS + SOC_PIXELS + SOC_MARKER_PIXELS; i++) {
        pixels.setPixelColor(i, pixels.Color(0, 0, 255));
    }
    pixels.show();
    delay(500);

    // Error lights - Yellow
    for(int i = TORQUE_PIXELS + SOC_PIXELS + SOC_MARKER_PIXELS; 
        i < TORQUE_PIXELS + SOC_PIXELS + SOC_MARKER_PIXELS + ERROR_PIXELS; i++) {
        pixels.setPixelColor(i, pixels.Color(255, 255, 0));
    }
    pixels.show();
    delay(500);

    // Temperature - Purple
    for(int i = TORQUE_PIXELS + SOC_PIXELS + SOC_MARKER_PIXELS + ERROR_PIXELS;
        i < TORQUE_PIXELS + SOC_PIXELS + SOC_MARKER_PIXELS + ERROR_PIXELS + TEMP_PIXELS; i++) {
        pixels.setPixelColor(i, pixels.Color(255, 0, 255));
    }
    pixels.show();
    delay(500);

    // Temperature markers - White
    for(int i = TORQUE_PIXELS + SOC_PIXELS + SOC_MARKER_PIXELS + ERROR_PIXELS + TEMP_PIXELS;
        i < TORQUE_PIXELS + SOC_PIXELS + SOC_MARKER_PIXELS + ERROR_PIXELS + TEMP_PIXELS + TEMP_MARKER_PIXELS; i++) {
        pixels.setPixelColor(i, pixels.Color(255, 255, 255));
    }
    pixels.show();
    delay(500);

    // Clear all
    pixels.clear();
    pixels.show();
    delay(500);

    debugPrint("Test pattern complete");

    // Initialize CAN with custom pins for ESP32
    SPI.begin(4, 5, 6, 7);  // SCK, MISO, MOSI, CS
    debugPrint("Initializing CAN...");
    
    if (can.reset() != MCP2515::ERROR_OK) {
        debugPrint("CAN reset failed!");
        return false;
    }

    if (can.setBitrate(CAN_500KBPS) != MCP2515::ERROR_OK) {
        debugPrint("CAN set bitrate failed!");
        return false;
    }

    if (can.setNormalMode() != MCP2515::ERROR_OK) {
        debugPrint("CAN set normal mode failed!");
        return false;
    }

    debugPrint("SpeedometerController initialization complete");
    return true;
}

void SpeedometerController::updateTorque(int16_t torque) {
    debugPrint("Updating torque: " + String(torque));
    
    // Clear previous torque pixels except LED 4
    for(int i = 0; i < 4; i++) pixels.setPixelColor(i, 0);
    for(int i = 5; i < TORQUE_PIXELS; i++) pixels.setPixelColor(i, 0);
    
    // LED 4 is always on (yellow)
    pixels.setPixelColor(4, pixels.Color(255, 255, 0));
    
    // Map torque (-100 to +400 Nm range)
    if (torque < 0) {
        // Regen: LED 3 for small regen, then adding 2, 1, 0 for stronger regen
        int numLeds = map(-torque, 0, 100, 0, 4);
        for(int i = 0; i < numLeds; i++) {
            pixels.setPixelColor(3 - i, pixels.Color(255, 0, 0));
        }
    } else if (torque > 0) {
        int ledPos = map(torque, 0, 400, 5, 19);
        setPixelRange(5, ledPos, pixels.Color(0, 255, 0)); // Green for positive torque
    }
}

void SpeedometerController::updateSOC(uint8_t percentage) {
    debugPrint("Updating SOC: " + String(percentage) + "%");
    int offset = TORQUE_PIXELS;  // Start after torque pixels
    
    // Clear previous SOC pixels
    for(int i = offset; i < offset + SOC_PIXELS; i++) {
        pixels.setPixelColor(i, 0);
    }
    
    // Light up SOC LEDs
    int ledsToLight = map(percentage, 0, 100, 0, SOC_PIXELS);
    debugPrint("SOC LEDs to light: " + String(ledsToLight));
    
    for (int i = 0; i < ledsToLight; i++) {
        uint32_t color;
        if (percentage > 60) color = pixels.Color(0, 255, 0);
        else if (percentage > 20) color = pixels.Color(255, 255, 0);
        else color = pixels.Color(255, 0, 0);
        pixels.setPixelColor(offset + i, color);
    }

    // Update SOC markers with brighter color
    offset += SOC_PIXELS;
    for (int i = 0; i < SOC_MARKER_PIXELS; i++) {
        pixels.setPixelColor(offset + i, pixels.Color(128, 128, 128));
    }
}

void SpeedometerController::updateErrorLights(uint16_t errorFlags) {
    debugPrint("Updating error lights: 0x" + String(errorFlags, HEX));
    int offset = TORQUE_PIXELS + SOC_PIXELS + SOC_MARKER_PIXELS;
    
    const uint32_t errorColors[] = {
        pixels.Color(255, 255, 255),    // Parking Brake - Red
        pixels.Color(255, 255, 255),    // Seat Buckle - Red
        pixels.Color(255, 255, 255),    // Battery Light - Orange
        pixels.Color(255, 255, 255),    // Diflock - Green
        pixels.Color(255, 255, 255),    // Fluid Low - Red
        pixels.Color(255, 255, 255),    // Brake System - Red
        pixels.Color(255, 255, 255),    // Battery Low - Red
        pixels.Color(255, 255, 255),    // Indicator - Green
        pixels.Color(255, 255, 255),    // Normal Lights - Green
        pixels.Color(255, 255, 255),    // High Beam - Blue
        pixels.Color(255, 255, 255),    // Rear Fog Light - Orange
        pixels.Color(255, 255, 255),    // Window Heating - Orange
        pixels.Color(255, 255, 255),    // Check Engine - Red
/*
        pixels.Color(255, 0, 0),     // Parking Brake - Red
        pixels.Color(255, 0, 0),     // Seat Buckle - Red
        pixels.Color(255, 165, 0),   // Battery Light - Orange
        pixels.Color(0, 255, 0),     // Diflock - Green
        pixels.Color(255, 0, 0),     // Fluid Low - Red
        pixels.Color(255, 0, 0),     // Brake System - Red
        pixels.Color(255, 0, 0),     // Battery Low - Red
        pixels.Color(0, 255, 0),     // Indicator - Green
        pixels.Color(0, 255, 0),     // Normal Lights - Green
        pixels.Color(0, 0, 255),     // High Beam - Blue
        pixels.Color(255, 165, 0),   // Rear Fog Light - Orange
        pixels.Color(255, 165, 0),   // Window Heating - Orange
        pixels.Color(255, 0, 0)      // Check Engine - Red
        */
    };

    // Update each error light
    for (int i = 0; i < ERROR_PIXELS; i++) {
        uint32_t color = (errorFlags & (1 << i)) ? errorColors[i] : 0;
        pixels.setPixelColor(offset + i, color);
    }
}

void SpeedometerController::updateTemperature(uint8_t temp) {
    debugPrint("Updating temperature: " + String(temp) + "°C");
    int offset = TORQUE_PIXELS + SOC_PIXELS + SOC_MARKER_PIXELS + ERROR_PIXELS;
    
    // Clear previous temperature pixels
    for(int i = offset; i < offset + TEMP_PIXELS; i++) {
        pixels.setPixelColor(i, 0);
    }
    
    // Map temperature (30-110°C) to LEDs
    int ledsToLight = map(constrain(temp, 30, 110), 30, 110, 0, TEMP_PIXELS);
    debugPrint("Temperature LEDs to light: " + String(ledsToLight));
    
    for (int i = 0; i < ledsToLight; i++) {
        uint32_t color;
        if (temp < 60) color = pixels.Color(0, 255, 0);
        else if (temp < 90) color = pixels.Color(255, 255, 0);
        else color = pixels.Color(255, 0, 0);
        pixels.setPixelColor(offset + i, color);
    }

    // Update temperature markers with brighter color
    offset += TEMP_PIXELS;
    for (int i = 0; i < TEMP_MARKER_PIXELS; i++) {
        pixels.setPixelColor(offset + i, pixels.Color(128, 128, 128));
    }
}

void SpeedometerController::setIllumination(uint8_t brightness) {
    int offset = TORQUE_PIXELS + SOC_PIXELS + SOC_MARKER_PIXELS + 
                ERROR_PIXELS + TEMP_PIXELS + TEMP_MARKER_PIXELS;
    
    uint32_t color = pixels.Color(brightness, brightness, brightness);
    for (int i = 0; i < ILLUMINATION_PIXELS; i++) {
        pixels.setPixelColor(offset + i, color);
    }
}

void SpeedometerController::show() {
    static unsigned long lastShowTime = 0;
    unsigned long currentTime = millis();
    
    // Update at approximately 100Hz
    if (currentTime - lastShowTime >= 10) {
        lastShowTime = currentTime;
        pixels.show();
    }
}

void SpeedometerController::clearPixelRange(int start, int count) {
    for (int i = 0; i < count; i++) {
        pixels.setPixelColor(start + i, 0);
    }
}

void SpeedometerController::setPixelRange(int start, int end, uint32_t color) {
    for (int i = start; i <= end; i++) {
        pixels.setPixelColor(i, color);
    }
}