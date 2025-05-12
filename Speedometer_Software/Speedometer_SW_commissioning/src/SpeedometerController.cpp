#include "SpeedometerController.h"
#include "DisplayController.h"
#include "SpeedometerPins.h"  // Include our pin definitions
#include <EEPROM.h>  // Added for odometer storage

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

bool SpeedometerController::begin() {
    debugPrint("Initializing SpeedometerController...");
    
    // Initialize EEPROM for ESP32
    if (!EEPROM.begin(64)) {
        debugPrint("Failed to initialize EEPROM");
    } else {
        debugPrint("EEPROM initialized");
        // Load odometer values from EEPROM
        loadOdometersFromEEPROM();
    }
    
    // Initialize NeoPixels
    pixels.begin();
    pixels.setBrightness(100);  // Start with lower brightness
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

    // Update the display with loaded odometer values
    if (display) {
        // Convert to integer values with one decimal for trip
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
    // Check if EEPROM has been initialized
    uint8_t initialized = EEPROM.read(EEPROM_INITIALIZED_ADDR);
    
    if (initialized == EEPROM_MAGIC_VALUE) {
        // Read the stored values
        EEPROM.get(EEPROM_TOTAL_ODO_ADDR, totalOdometer);
        EEPROM.get(EEPROM_TRIP_ODO_ADDR, tripOdometer);
        
        debugPrint("Loaded from EEPROM - Total: " + String(totalOdometer) + 
                  " km, Trip: " + String(tripOdometer) + " km");
    } else {
        // Initialize EEPROM with default values
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
    
    // Save values every 10 minutes or when there's a significant change
    if (currentTime - lastSaveTime >= 600000) {  // 10 minutes
        EEPROM.put(EEPROM_TOTAL_ODO_ADDR, totalOdometer);
        EEPROM.put(EEPROM_TRIP_ODO_ADDR, tripOdometer);
        EEPROM.commit();
        
        lastSaveTime = currentTime;
        debugPrint("Odometer values saved to EEPROM");
    }
}

void SpeedometerController::resetTripOdometer() {
    tripOdometer = 0.0;
    
    // Save to EEPROM immediately
    EEPROM.put(EEPROM_TRIP_ODO_ADDR, tripOdometer);
    EEPROM.commit();
    
    // Update display
    if (display) {
        display->displayTripKm(0, 0);
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
                case CANIds::DMC_STATUS:
                    // Process DMC status message (torque, speed)
                    torqueAvailable = ((buf[2] << 8) | buf[3]) * 0.01;
                    torqueActual = ((buf[4] << 8) | buf[5]) * 0.01;
                    speedActual = static_cast<float>(static_cast<int16_t>((buf[6] << 8) | buf[7]));
                    updateVehicleSpeed();
                    break;
                    
                case CANIds::DMC_POWER: {
                    // Process DMC power message
                    dcVoltageAct = ((buf[0] << 8) | buf[1]) * 0.1;
                    
                    // FIX: Properly handle signed current using int16_t cast
                    int16_t currentRaw = (buf[2] << 8) | buf[3];
                    dcCurrentAct = static_cast<float>(currentRaw) * 0.1;
                    
                    acCurrentAct = static_cast<float>(static_cast<int16_t>((buf[4] << 8) | buf[5])) * 0.25;
                    mechPower = static_cast<int32_t>(static_cast<int16_t>((buf[6] << 8) | buf[7])) * 16;
                    
                    // Debug print the raw current and converted value to verify sign handling
                    debugPrint("Raw current: 0x" + String(currentRaw, HEX) + 
                              ", Converted: " + String(dcCurrentAct) + "A");
                    
                    // Update torque display based on DC current
                    updateTorque(static_cast<int16_t>(dcCurrentAct));
                    break;
                }
                    
                case CANIds::DMC_TEMP: {
                    // Process DMC temperature message using correct scaling factors
                    // SigDMC_TempInv: 0.5°C/bit, signed
                    int16_t tempInvRaw = (buf[0] << 8) | buf[1];
                    tempInverter = tempInvRaw * 0.5;
                    
                    // SigDMC_TempMot: 0.5°C/bit, signed
                    int16_t tempMotRaw = (buf[2] << 8) | buf[3];
                    tempMotor = tempMotRaw * 0.5;
                    
                    // SigDMC_TempSys: 1°C/bit, unsigned, with -50°C offset
                    tempSystem = buf[4] - 50;
                    
                    // Show the highest temperature of the three
                    float highestTemp = max(tempInverter, max(tempMotor, static_cast<float>(tempSystem)));
                    
                    // Debug print all temperatures to diagnose the issue
                    debugPrint("Temp values - Inverter: " + String(tempInverter) + 
                               "°C, Motor: " + String(tempMotor) + 
                               "°C, System: " + String(tempSystem) + 
                               "°C, Highest: " + String(highestTemp) + "°C");
                               
                    // Explicitly constrain the temperature to reasonable values before updating
                    // This prevents any potential undefined behavior from bad CAN data
                    float constrainedTemp = constrain(highestTemp, 0, 100);
                    updateTemperature(static_cast<uint8_t>(constrainedTemp));
                    break;
                }
                    
                case CANIds::BMS_STATUS:
                    // Process BMS status message
                    soc = buf[0] / 2;  // Scale to percentage
                    bmsVoltage = (buf[2] | (buf[1] << 8)) / 10;
                    // FIX: Handle signed current properly
                    bmsCurrent = static_cast<int16_t>(buf[4] | (buf[3] << 8));
                    updateSOC(soc);
                    break;
                    
                case CANIds::DMC_CTRL: {
                    // Parse gear state from enablePosSpeed and enableNegSpeed
                    bool enablePosSpeed = buf[0] & 0x01;  // Bit 0
                    bool enableNegSpeed = buf[0] & 0x02;  // Bit 1
                    
                    GearState newGearState;
                    
                    // Fix gear state determination logic
                    if (enableNegSpeed && !enablePosSpeed) {
                        // Only negative speed enabled = Drive
                        newGearState = GEAR_DRIVE;
                    } else if (enablePosSpeed && !enableNegSpeed) {
                        // Only positive speed enabled = Reverse
                        newGearState = GEAR_REVERSE;
                    } else {
                        // Both off or both on = Neutral
                        newGearState = GEAR_NEUTRAL;
                    }
                    
                    // Only update display if gear state has changed
                    if (newGearState != gearState) {
                        gearState = newGearState;
                        
                        // Update the display with the new gear state
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
    
    // Update odometer based on vehicle speed
    updateOdometer();
}

void SpeedometerController::updateOdometer() {
    unsigned long currentTime = millis();
    
    // Update odometer every second
    if (currentTime - lastOdometerUpdate >= 1000) {
        // Calculate distance traveled in kilometers
        // vehicleSpeed is in kph, so we need to convert to km/s and multiply by elapsed time
        float elapsedHours = (currentTime - lastOdometerUpdate) / 3600000.0;  // Convert ms to hours
        float distanceTraveled = vehicleSpeed * elapsedHours;  // Distance in km
        
        // Update odometers
        totalOdometer += distanceTraveled;
        tripOdometer += distanceTraveled;
        
        // Update the display
        if (display) {
            // Convert to integer values with one decimal for trip
            unsigned long totalKm = static_cast<unsigned long>(totalOdometer);
            unsigned long tripKm = static_cast<unsigned long>(tripOdometer);
            uint8_t tripDecimal = static_cast<uint8_t>((tripOdometer - tripKm) * 10);
            
            display->displayTotalKm(totalKm);
            display->displayTripKm(tripKm, tripDecimal);
        }
        
        lastOdometerUpdate = currentTime;
        
        // Save odometer values to EEPROM periodically
        saveOdometersToEEPROM();
    }
}

void SpeedometerController::updateVehicleSpeed() {
    unsigned long currentTime = millis();
    
    // Only update speed display if significant time has passed or speed has changed
    if (currentTime - lastSpeedUpdate >= 100 || abs(speedActual - lastSpeedActual) > 10) {
        // Calculate vehicle speed from motor speed using VCU's conversion formula
        // This is based on the calculation in VehicleControl::calculateVehicleSpeed()
        const float NORMAL_RATIO = 1.2f;  // From VehicleParams::Transmission::NORMAL_RATIO
        const float DIFF_RATIO = 3.9f;    // From VehicleParams::Transmission::DIFF_RATIO
        const float WHEEL_CIRC = 2.08f;   // From VehicleParams::Transmission::WHEEL_CIRC (meters)
        
        // Convert RPM to kph
        vehicleSpeed = abs(speedActual) * 60.0f / NORMAL_RATIO / DIFF_RATIO * WHEEL_CIRC/1000.0f;
        
        // Low-pass filter for smoother display
        currentSpeed = (currentSpeed * 0.7) + (vehicleSpeed * 0.3);
        
        // Update the speed display
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
    
    // Only update if the value has changed significantly or enough time has passed
    if (abs(current - lastCurrent) > 20 || (currentTime - lastUpdate) > 200) {
        lastCurrent = current;
        lastUpdate = currentTime;
        
        debugPrint("Updating torque based on current: " + String(current) + "A");
        
        // Clear all torque pixels first for a clean slate
        for(int i = 0; i < TORQUE_PIXELS; i++) {
            pixels.setPixelColor(i, 0);
        }
        
        // Center LED (LED 4) is GREEN at idle
        pixels.setPixelColor(4, pixels.Color(0, 255, 0));  // Green for center at all times
        
        if (current < -10) {  // Negative current/Regen - sweep UP from center (LEDs 3-0)
            // Map from -10 to -450A to 0 to 4 LEDs (excluding center LED which is always on)
            int numLeds = map(constrain(-current, 10, 450), 10, 450, 1, 4);
            
            // Light up the LEDs in sequence from center upward with GREEN for regen
            // Start at LED 3 (just above center) and work upward to LED 0
            for(int i = 0; i < numLeds; i++) {
                pixels.setPixelColor(3 - i, pixels.Color(0, 255, 0)); // Green for regen
            }
        } else if (current > 10) {  // Positive current/Driving - sweep DOWN from center (LEDs 5-19)
            // Map from 10 to 450A to 0 to 15 LEDs
            int numLeds = map(constrain(current, 10, 450), 10, 450, 1, 15);
            
            // Light up the LEDs in sequence from center downward with ORANGE for power
            // Start at LED 5 (just below center) and work downward
            for(int i = 0; i < numLeds; i++) {
                pixels.setPixelColor(5 + i, pixels.Color(255, 165, 0)); // Orange for driving (255, 165, 0)
            }
        }
        // If between -10 and 10, only the center green LED remains lit
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
        uint32_t color = pixels.Color(0, 255, 0);  // Green for all charge levels
        pixels.setPixelColor(offset + i, color);
    }

    // Restore SOC markers with dim green
    offset += SOC_PIXELS;
    for (int i = 0; i < SOC_MARKER_PIXELS; i++) {
        pixels.setPixelColor(offset + i, pixels.Color(0, 64, 0));  // Dim green for markers
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

    // Update each error light
    for (int i = 0; i < ERROR_PIXELS; i++) {
        uint32_t color = (errorFlags & (1 << i)) ? errorColors[i] : 0;
        pixels.setPixelColor(offset + i, color);
    }
}

void SpeedometerController::updateTemperature(uint8_t temp) {
    debugPrint("Updating temperature display: " + String(temp) + "°C");
    int offset = TORQUE_PIXELS + SOC_PIXELS + SOC_MARKER_PIXELS + ERROR_PIXELS;
    
    // Clear previous temperature pixels
    for(int i = offset; i < offset + TEMP_PIXELS; i++) {
        pixels.setPixelColor(i, 0);
    }
    
    // Map temperature (0-110°C) to LEDs - full range as specified
    int ledsToLight = map(constrain(temp, 0, 110), 0, 110, 0, TEMP_PIXELS);
    debugPrint("Temperature LEDs to light: " + String(ledsToLight));
    
    for (int i = 0; i < ledsToLight; i++) {
        uint32_t color = pixels.Color(0, 255, 0);  // Green for all temperature levels
        pixels.setPixelColor(offset + i, color);
    }

    // Restore temperature markers with a dim green color (instead of white)
    offset += TEMP_PIXELS;
    for (int i = 0; i < TEMP_MARKER_PIXELS; i++) {
        pixels.setPixelColor(offset + i, pixels.Color(0, 64, 0));  // Dim green for markers
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
    
    // Update at approximately 50Hz (20ms)
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