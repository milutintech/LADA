#include "SpeedometerController.h"
#include "DisplayController.h"
#include "SpeedometerPins.h"
#include <EEPROM.h>

// Define EEPROM addresses for storing odometer values
#define EEPROM_TOTAL_ODO_ADDR 0
#define EEPROM_TRIP_ODO_ADDR 8
#define EEPROM_INITIALIZED_ADDR 16
#define EEPROM_MAGIC_VALUE 0xAB

extern DisplayController* display;

// CAN message IDs - aligned with VCU codebase
namespace CANIds {
    constexpr uint16_t BSC_VAL = 0x26A;      // BSC values (hvVoltageAct, lvVoltageAct, hvCurrentAct, lvCurrentAct)
    constexpr uint16_t DMC_STATUS = 0x258;    // DMC Status (torqueAvailable, torqueActual, speedActual)
    constexpr uint16_t DMC_POWER = 0x259;     // DMC Power (dcVoltageAct, dcCurrentAct, acCurrentAct, mechPower)
    constexpr uint16_t DMC_TEMP = 0x458;      // DMC Temperature (tempInverter, tempMotor, tempSystem)
    constexpr uint16_t DMC_ERRORS = 0x25A;    // DMC Error flags
    constexpr uint16_t BMS_STATUS = 0x010;    // BMS Status (soc, voltage, current)
    constexpr uint16_t DMC_CTRL = 0x210;      // DMC Control (enablePosSpeed, enableNegSpeed for gear detection)
}

SpeedometerController::SpeedometerController() : 
    pixels(TOTAL_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800),
    canBus(nullptr),
    customSPI(nullptr),
    lastOdometerUpdate(0),
    totalOdometer(0),
    tripOdometer(0),
    lastSpeedUpdate(0),
    currentSpeed(0),
    lastDistance(0),
    lastSpeedActual(0) {
    
    // Initialize custom SPI interface
    customSPI = new SPIClass(HSPI);
    canBus = new mcp2515_can(CAN_CS_PIN);
}

void SpeedometerController::debugPrint(String msg) {
    #if DEBUG_PRINTS
    Serial.println(msg);
    #endif
}

// HSL to RGB conversion function
uint32_t SpeedometerController::hslToRgb(float h, float s, float l) {
    float c = (1.0 - abs(2.0 * l - 1.0)) * s;
    float x = c * (1.0 - abs(fmod(h / 60.0, 2.0) - 1.0));
    float m = l - c / 2.0;
    
    float r, g, b;
    
    if (h >= 0 && h < 60) {
        r = c; g = x; b = 0;
    } else if (h >= 60 && h < 120) {
        r = x; g = c; b = 0;
    } else if (h >= 120 && h < 180) {
        r = 0; g = c; b = x;
    } else if (h >= 180 && h < 240) {
        r = 0; g = x; b = c;
    } else if (h >= 240 && h < 300) {
        r = x; g = 0; b = c;
    } else {
        r = c; g = 0; b = x;
    }
    
    // Convert to 0-255 range and apply brightness adjustment
    uint8_t red = (uint8_t)((r + m) * 255);
    uint8_t green = (uint8_t)((g + m) * 255);
    uint8_t blue = (uint8_t)((b + m) * 255);
    
    return pixels.Color(red, green, blue);
}

// Get temperature-based color (green to red fade starting at 80°C)
uint32_t SpeedometerController::getTemperatureColor(uint8_t temp) {
    float hue;
    
    if (temp <= 90) {
        // Green for temps up to 90°C
        hue = 120.0; // Pure green
    } else {
        // Fade from green (120°) to red (0°) for temps 90-110°C
        float tempRange = constrain(temp, 90, 110);
        hue = map(tempRange, 90, 110, 120, 0); // Linear interpolation from green to red
    }
    
    // Use full saturation and moderate lightness for good visibility
    return hslToRgb(hue, 1.0, 0.5);
}

// Get SOC-based color (green to red fade from 25% to 5%, then stay red)
uint32_t SpeedometerController::getSOCColor(uint8_t percentage) {
    float hue;
    
    if (percentage >= 25) {
        // Green for SOC 25% and above
        hue = 120.0; // Pure green
    } else if (percentage >= 15) {
        // Fade from green (120°) to red (0°) for SOC 25% to 15%
        hue = map(percentage, 15, 25, 0, 120); // Linear interpolation from red to green
    } else {
        // Red for SOC below 15%
        hue = 0.0; // Pure red
    }
    
    // Use full saturation and moderate lightness for good visibility
    return hslToRgb(hue, 1.0, 0.5);
}


bool SpeedometerController::begin() {
    debugPrint("Initializing SpeedometerController...");
    
    // Initialize EEPROM for ESP32
    if (!EEPROM.begin(64)) {
        debugPrint("Failed to initialize EEPROM");
    } else {
        debugPrint("EEPROM initialized");
        loadOdometersFromEEPROM();
    }
    
    // Initialize NeoPixels
    pixels.begin();
    pixels.setBrightness(100);
    pixels.clear();
    pixels.show();
    delay(100);

    // Test pattern - light up each section in different colors
    debugPrint("Running startup test pattern...");
    
    // Torque section - Red
    for(int i = 0; i < TORQUE_PIXELS; i++) {
        pixels.setPixelColor(i, pixels.Color(255, 0, 0));
    }
    pixels.show();
    delay(300);

    // SOC section - Green
    for(int i = TORQUE_PIXELS; i < TORQUE_PIXELS + SOC_PIXELS; i++) {
        pixels.setPixelColor(i, pixels.Color(0, 255, 0));
    }
    pixels.show();
    delay(300);

    // SOC markers - Blue
    for(int i = TORQUE_PIXELS + SOC_PIXELS; i < TORQUE_PIXELS + SOC_PIXELS + SOC_MARKER_PIXELS; i++) {
        pixels.setPixelColor(i, pixels.Color(0, 0, 255));
    }
    pixels.show();
    delay(300);

    // Error lights - Yellow
    for(int i = TORQUE_PIXELS + SOC_PIXELS + SOC_MARKER_PIXELS; 
        i < TORQUE_PIXELS + SOC_PIXELS + SOC_MARKER_PIXELS + ERROR_PIXELS; i++) {
        pixels.setPixelColor(i, pixels.Color(255, 255, 0));
    }
    pixels.show();
    delay(300);

    // Temperature - Purple
    for(int i = TORQUE_PIXELS + SOC_PIXELS + SOC_MARKER_PIXELS + ERROR_PIXELS;
        i < TORQUE_PIXELS + SOC_PIXELS + SOC_MARKER_PIXELS + ERROR_PIXELS + TEMP_PIXELS; i++) {
        pixels.setPixelColor(i, pixels.Color(255, 0, 255));
    }
    pixels.show();
    delay(300);

    // Temperature markers - White
    for(int i = TORQUE_PIXELS + SOC_PIXELS + SOC_MARKER_PIXELS + ERROR_PIXELS + TEMP_PIXELS;
        i < TORQUE_PIXELS + SOC_PIXELS + SOC_MARKER_PIXELS + ERROR_PIXELS + TEMP_PIXELS + TEMP_MARKER_PIXELS; i++) {
        pixels.setPixelColor(i, pixels.Color(255, 255, 255));
    }
    pixels.show();
    delay(300);

    // Illumination - White Dim
    for(int i = TORQUE_PIXELS + SOC_PIXELS + SOC_MARKER_PIXELS + ERROR_PIXELS + TEMP_PIXELS + TEMP_MARKER_PIXELS;
        i < TOTAL_PIXELS; i++) {
        pixels.setPixelColor(i, pixels.Color(100, 100, 100));
    }
    pixels.show();
    delay(300);

    // Clear all
    pixels.clear();
    pixels.show();
    delay(300);

    debugPrint("Test pattern complete");

    // Initialize CAN with custom pins for ESP32
    if (!customSPI) {
        debugPrint("SPI not initialized!");
        return false;
    }
    
    // Initialize SPI with correct pins from SpeedometerPins.h
    customSPI->begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, CAN_CS_PIN);
    canBus->setSPI(customSPI);
    
    // Set up interrupt pin if using
    pinMode(CAN_INT_PIN, INPUT_PULLUP);
    
    debugPrint("Initializing CAN...");
    
    uint8_t retries = 0;
    const uint8_t MAX_RETRIES = 5;
    
    while (CAN_OK != canBus->begin(CAN_500KBPS)) {
        debugPrint("CAN BUS Shield init fail");
        delay(100);
        if (++retries >= MAX_RETRIES) {
            debugPrint("CAN init failed after max retries");
            return false;
        }
    }
    
    debugPrint("CAN init OK!");

    // Initial values
    torque = 0;
    soc = 50;
    errorFlags = 0;
    temperature = 30;
    speedActual = 0;
    dcCurrentAct = 0;
    vehicleSpeed = 0;
    gearState = GEAR_NEUTRAL;
    
    // Initialize BSC values
    hvVoltageAct = 0.0;
    lvVoltageAct = 12.0;  // Default to normal 12V
    hvCurrentAct = 0.0;
    lvCurrentAct = 0.0;
    
    // Initialize DMC error flag
    dmcHasErrors = false;

    // Update the display with loaded odometer values
    if (display) {
        unsigned long totalKm = static_cast<unsigned long>(totalOdometer);
        unsigned long tripKm = static_cast<unsigned long>(tripOdometer);
        uint8_t tripDecimal = static_cast<uint8_t>((tripOdometer - tripKm) * 10);
        
        display->displayTotalKm(totalKm);
        display->displayTripKm(tripKm, tripDecimal);
    }

    debugPrint("SpeedometerController initialization complete");
    return true;
}

void SpeedometerController::loadOdometersFromEEPROM() {
    uint8_t initialized = EEPROM.read(EEPROM_INITIALIZED_ADDR);
    
    if (initialized == EEPROM_MAGIC_VALUE) {
        EEPROM.get(EEPROM_TOTAL_ODO_ADDR, totalOdometer);
        EEPROM.get(EEPROM_TRIP_ODO_ADDR, tripOdometer);
        
        debugPrint("Loaded from EEPROM - Total: " + String(totalOdometer) + 
                  " km, Trip: " + String(tripOdometer) + " km");
    } else {
        totalOdometer = 0.0;
        tripOdometer = 0.0;
        
        EEPROM.put(EEPROM_TOTAL_ODO_ADDR, totalOdometer);
        EEPROM.put(EEPROM_TRIP_ODO_ADDR, tripOdometer);
        EEPROM.write(EEPROM_INITIALIZED_ADDR, EEPROM_MAGIC_VALUE);
        EEPROM.commit();
        
        debugPrint("EEPROM initialized with default odometer values");
    }
}

void SpeedometerController::saveOdometersToEEPROM() {
    static unsigned long lastSaveTime = 0;
    unsigned long currentTime = millis();
    
    if (currentTime - lastSaveTime >= 1000) {  // Every 1 second instead of 10 minutes
        EEPROM.put(EEPROM_TOTAL_ODO_ADDR, totalOdometer);
        EEPROM.put(EEPROM_TRIP_ODO_ADDR, tripOdometer);
        EEPROM.commit();
        
        lastSaveTime = currentTime;
        debugPrint("Odometer values saved to EEPROM");
    }
}

void SpeedometerController::resetTripOdometer() {
    tripOdometer = 0.0;
    
    EEPROM.put(EEPROM_TRIP_ODO_ADDR, tripOdometer);
    EEPROM.commit();
    
    if (display) {
        //display->displayTripKm(0, 0);
    }
    
    debugPrint("Trip odometer reset");
}

void SpeedometerController::processCANMessages() {
    uint8_t len;
    uint8_t buf[8];
    
    while (CAN_MSGAVAIL == canBus->checkReceive()) {
        if (canBus->readMsgBuf(&len, buf) == CAN_OK) {
            uint32_t id = canBus->getCanId();
            
            switch (id) {
                case CANIds::BSC_VAL: {
                    // Process BSC message (0x26A) for LV voltage
                    hvVoltageAct = ((buf[0] << 8) | buf[1]) * 0.1f;
                    lvVoltageAct = buf[2] * 0.1f;
                    hvCurrentAct = (((buf[3] << 8) | buf[4]) * 0.1f) - 25.0f;
                    lvCurrentAct = ((buf[5] << 8) | buf[6]) - 280.0f;
                    
                    debugPrint("BSC Data - HV: " + String(hvVoltageAct) + "V, LV: " + String(lvVoltageAct) + "V");
                    break;
                }
                
                case CANIds::DMC_STATUS:
                    torqueAvailable = ((buf[2] << 8) | buf[3]) * 0.01;
                    torqueActual = ((buf[4] << 8) | buf[5]) * 0.01;
                    speedActual = static_cast<float>(static_cast<int16_t>((buf[6] << 8) | buf[7]));
                    updateVehicleSpeed();
                    break;
                    
                case CANIds::DMC_POWER: {
                    dcVoltageAct = ((buf[0] << 8) | buf[1]) * 0.1;
                    
                    int16_t currentRaw = (buf[2] << 8) | buf[3];
                    dcCurrentAct = static_cast<float>(currentRaw) * 0.1;
                    
                    acCurrentAct = static_cast<float>(static_cast<int16_t>((buf[4] << 8) | buf[5])) * 0.25;
                    mechPower = static_cast<int32_t>(static_cast<int16_t>((buf[6] << 8) | buf[7])) * 16;
                    
                    debugPrint("Raw current: 0x" + String(currentRaw, HEX) + 
                              ", Converted: " + String(dcCurrentAct) + "A");
                    
                    updateTorque(static_cast<int16_t>(dcCurrentAct));
                    break;
                }
                    
                case CANIds::DMC_TEMP: {
                    int16_t tempInvRaw = (buf[0] << 8) | buf[1];
                    tempInverter = tempInvRaw * 0.5;
                    
                    int16_t tempMotRaw = (buf[2] << 8) | buf[3];
                    tempMotor = tempMotRaw * 0.5;
                    
                    tempSystem = buf[4] - 50;
                    
                    float highestTemp = max(tempInverter, max(tempMotor, static_cast<float>(tempSystem)));
                    
                    debugPrint("Temp values - Inverter: " + String(tempInverter) + 
                               "°C, Motor: " + String(tempMotor) + 
                               "°C, System: " + String(tempSystem) + 
                               "°C, Highest: " + String(highestTemp) + "°C");
                               
                    float constrainedTemp = constrain(highestTemp, 0, 100);
                    updateTemperature(static_cast<uint8_t>(constrainedTemp));
                    break;
                }

                case CANIds::DMC_ERRORS: {
                    // Process DMC error message (0x25A)
                    uint64_t errorBits = 0;
                    
                    // Combine all 8 bytes into 64-bit value
                    for (int i = 0; i < 8; i++) {
                        errorBits |= ((uint64_t)buf[i] << (i * 8));
                    }
                    
                    // Check if any error bits (0-47) are set
                    uint64_t errorMask = 0x0000FFFFFFFFFFFF; // Mask for bits 0-47
                    dmcHasErrors = (errorBits & errorMask) != 0;
                    
                    debugPrint("DMC Errors: 0x" + String((uint32_t)(errorBits >> 32), HEX) + 
                              String((uint32_t)errorBits, HEX) + ", Has Errors: " + String(dmcHasErrors));
                    break;
                }
                    
                case CANIds::BMS_STATUS:
                    soc = buf[0] / 2;
                    bmsVoltage = (buf[2] | (buf[1] << 8)) / 10;
                    bmsCurrent = static_cast<int16_t>(buf[4] | (buf[3] << 8));
                    updateSOC(soc);
                    break;
                    
                case CANIds::DMC_CTRL: {
                    bool enablePosSpeed = buf[0] & 0x01;
                    bool enableNegSpeed = buf[0] & 0x02;
                    
                    GearState newGearState;
                    
                    if (enableNegSpeed && !enablePosSpeed) {
                        newGearState = GEAR_DRIVE;
                    } else if (enablePosSpeed && !enableNegSpeed) {
                        newGearState = GEAR_REVERSE;
                    } else {
                        newGearState = GEAR_NEUTRAL;
                    }
                    
                    if (newGearState != gearState) {
                        gearState = newGearState;
                        
                        if (display) {
                            DisplayController::DriveMode displayMode;
                            switch (gearState) {
                                case GEAR_DRIVE:
                                    displayMode = DisplayController::MODE_D;
                                    break;
                                case GEAR_REVERSE:
                                    displayMode = DisplayController::MODE_R;
                                    break;
                                case GEAR_NEUTRAL:
                                default:
                                    displayMode = DisplayController::MODE_N;
                                    break;
                            }
                            display->displayDriveMode(displayMode);
                        }
                        
                        debugPrint("Gear state changed to: " + String(gearState));
                    }
                    break;
                }
            }
        }
    }
    
    updateOdometer();
}

void SpeedometerController::updateOdometer() {
    unsigned long currentTime = millis();
    
    if (currentTime - lastOdometerUpdate >= 1000) {
        float elapsedHours = (currentTime - lastOdometerUpdate) / 3600000.0;
        float distanceTraveled = vehicleSpeed * elapsedHours;
        
        totalOdometer += distanceTraveled;
        tripOdometer += distanceTraveled;
        
        if (display) {
            unsigned long totalKm = static_cast<unsigned long>(totalOdometer);
            display->displayTotalKm(totalKm);
            
            // Don't update trip display here anymore - SOC uses it now
            // display->displayTripKm(tripKm, tripDecimal); // REMOVED
        }
        
        lastOdometerUpdate = currentTime;
        saveOdometersToEEPROM();
    }
}

void SpeedometerController::updateVehicleSpeed() {
    unsigned long currentTime = millis();
    
    if (currentTime - lastSpeedUpdate >= 100 || abs(speedActual - lastSpeedActual) > 10) {
        const float NORMAL_RATIO = 1.2f;
        const float DIFF_RATIO = 3.9f;
        const float WHEEL_CIRC = 2.08f;
        
        vehicleSpeed = abs(speedActual) * 60.0f / NORMAL_RATIO / DIFF_RATIO * WHEEL_CIRC/1000.0f;
        
        currentSpeed = (currentSpeed * 0.7) + (vehicleSpeed * 0.3);
        
        if (display) {
            display->displaySpeed(static_cast<unsigned int>(currentSpeed));
        }
        Serial.println(speedActual);
        lastSpeedUpdate = currentTime;
        lastSpeedActual = speedActual;
    }
}

void SpeedometerController::updateTorque(int16_t current) {
    static int16_t lastCurrent = 0;
    static unsigned long lastUpdate = 0;
    unsigned long currentTime = millis();
    
    if (abs(current - lastCurrent) > 20 || (currentTime - lastUpdate) > 200) {
        lastCurrent = current;
        lastUpdate = currentTime;
        
        debugPrint("Updating torque based on current: " + String(current) + "A");
        
        for(int i = 0; i < TORQUE_PIXELS; i++) {
            pixels.setPixelColor(i, 0);
        }
        
        pixels.setPixelColor(4, pixels.Color(0, 255, 0));
        
        if (current < -10) {
            int numLeds = map(constrain(-current, 10, 450), 10, 450, 1, 4);
            
            for(int i = 0; i < numLeds; i++) {
                pixels.setPixelColor(3 - i, pixels.Color(0, 255, 0));
            }
        } else if (current > 10) {
            int numLeds = map(constrain(current, 10, 450), 10, 450, 1, 15);
            
            for(int i = 0; i < numLeds; i++) {
                pixels.setPixelColor(5 + i, pixels.Color(255, 165, 0));
            }
        }
    }
}

void SpeedometerController::updateSOC(uint8_t percentage) {
    debugPrint("Updating SOC: " + String(percentage) + "%");
    
    // Use the trip display to show SOC instead of the dedicated SOC pixels
    if (display) {
        // Display SOC as a percentage with one decimal place
        // For example: SOC 87% would show as 87.0
        display->displayTripKm(percentage, 0);
    }
    
    // Optional: Still update the SOC pixels for backup/additional indication
    int offset = TORQUE_PIXELS;
    
    // Clear all SOC pixels first
    for(int i = offset; i < offset + SOC_PIXELS; i++) {
        pixels.setPixelColor(i, 0);
    }
    
    int ledsToLight = map(percentage, 0, 100, 0, SOC_PIXELS);
    debugPrint("SOC LEDs to light: " + String(ledsToLight));
    
    // Light up LEDs with color-coded values
    for (int i = 0; i < ledsToLight; i++) {
        uint32_t color = getSOCColor(percentage);
        pixels.setPixelColor(offset + i, color);
    }

    // Update SOC marker pixels with dimmed green
    offset += SOC_PIXELS;
    for (int i = 0; i < SOC_MARKER_PIXELS; i++) {
        pixels.setPixelColor(offset + i, pixels.Color(0, 64, 0));
    }
}

void SpeedometerController::updateErrorLights(uint16_t errorFlags) {
    debugPrint("Updating error lights: 0x" + String(errorFlags, HEX));
    int offset = TORQUE_PIXELS + SOC_PIXELS + SOC_MARKER_PIXELS;
    
    const uint32_t errorColors[] = {
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
    };

    for (int i = 0; i < ERROR_PIXELS; i++) {
        uint32_t color = (errorFlags & (1 << i)) ? errorColors[i] : 0;
        pixels.setPixelColor(offset + i, color);
    }
}

void SpeedometerController::updateTemperature(uint8_t temp) {
    debugPrint("Updating temperature display: " + String(temp) + "°C");
    int offset = TORQUE_PIXELS + SOC_PIXELS + SOC_MARKER_PIXELS + ERROR_PIXELS;
    
    // Clear all temperature pixels first
    for(int i = offset; i < offset + TEMP_PIXELS; i++) {
        pixels.setPixelColor(i, 0);
    }
    
    int ledsToLight = map(constrain(temp, 0, 110), 0, 110, 0, TEMP_PIXELS);
    debugPrint("Temperature LEDs to light: " + String(ledsToLight));
    
    // Light up LEDs with color-coded temperature values
    for (int i = 0; i < ledsToLight; i++) {
        uint32_t color = getTemperatureColor(temp);
        pixels.setPixelColor(offset + i, color);
    }

    // Update temperature marker pixels with dimmed green
    offset += TEMP_PIXELS;
    for (int i = 0; i < TEMP_MARKER_PIXELS; i++) {
        pixels.setPixelColor(offset + i, pixels.Color(0, 64, 0));
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
    
    if (currentTime - lastShowTime >= 20) {
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