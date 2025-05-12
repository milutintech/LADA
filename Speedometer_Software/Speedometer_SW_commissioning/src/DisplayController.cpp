#include "DisplayController.h"

extern const uint8_t SEVEN_SEG_PATTERNS[];
extern const uint16_t FOURTEEN_SEG_PATTERNS[];

DisplayController::DisplayController() {
    displays = new TLC59108*[NUM_ICS];
    icAddresses = new byte[NUM_ICS];
    validDisplays = new bool[NUM_ICS];
    
    // Set default addresses (0x40 + index) and assume all displays are valid by default
    for (int i = 0; i < NUM_ICS; i++) {
        icAddresses[i] = 0x40 + i;
        validDisplays[i] = true;
    }
    
    // Apply custom addresses for specific digit positions
    // Digit 1: 1001010 (binary) = 0x4A (hex)
    // Digit 10: 1000101 (binary) = 0x45 (hex)
    // Digit 100: 1001100 (binary) = 0x4C (hex)
    icAddresses[SPEED_START] = 0x4A;     // Ones digit
    icAddresses[SPEED_START+1] = 0x45;   // Tens digit
    icAddresses[SPEED_START+2] = 0x4C;   // Hundreds digit
    
    // Mark the 100k display as removed/invalid
    validDisplays[TOTAL_KM_START + 5] = false;
    
    // Initialize all valid displays with their corresponding addresses
    for (int i = 0; i < NUM_ICS; i++) {
        if (!validDisplays[i]) {
            Serial.printf("Skipping display at index %d (marked as removed)\n", i);
            displays[i] = nullptr;
            continue;
        }
        
        displays[i] = new TLC59108(Wire, icAddresses[i]);
        
        // Begin communication with the display
        if (!displays[i]->begin()) {
            Serial.printf("Failed to initialize display at address 0x%02X (index %d)\n", 
                         icAddresses[i], i);
        } else {
            Serial.printf("Successfully initialized display at address 0x%02X (index %d)\n", 
                         icAddresses[i], i);
        }
        
        displays[i]->setOutputMode(2);
        for (int j = 0; j < 8; j++) {
            displays[i]->setBrightness(j, 0);
        }
    }
}

DisplayController::~DisplayController() {
    for (int i = 0; i < NUM_ICS; i++) {
        if (displays[i] != nullptr) {
            delete displays[i];
        }
    }
    delete[] displays;
    delete[] icAddresses;
    delete[] validDisplays;
}

void DisplayController::displayTotalKm(unsigned long km) {
    // We'll display only 5 digits instead of 6, as the 100k digit is removed
    for (uint8_t i = 0; i < 5; i++) {
        uint8_t digit = km % 10;
        setSevenSegment(TOTAL_KM_START + i, digit, false);
        km /= 10;
    }
    
    // Skip the 6th digit (100k place) as it's been removed
}

void DisplayController::displayTripKm(unsigned long km, uint8_t decimal) {
    // Display decimal digit
    setSevenSegment(TRIP_KM_START + 0, decimal, false);
    
    // Display main digits
    for (uint8_t i = 0; i < 3; i++) {
        uint8_t digit = km % 10;
        setSevenSegment(TRIP_KM_START + (i+1), digit, i == 0);
        km /= 10;
    }
}

void DisplayController::displaySpeed(unsigned int speed) {
    // Special case for 0 - Fixed to avoid flickering
    if (speed == 0) {
        // Always display a zero in the ones place rather than turning all off
        setSevenSegment(SPEED_START, 0, false);
        
        // Turn off tens and hundreds digits
        if (displays[SPEED_START + 1] != nullptr)
            displays[SPEED_START + 1]->setAllBrightness(0); // Tens digit (0x45)
        if (displays[SPEED_START + 2] != nullptr)
            displays[SPEED_START + 2]->setAllBrightness(0); // Hundreds digit (0x4C)
        return;
    }

    unsigned int temp = speed;
    int numDigits = 0;
    
    // Count number of digits
    while (temp > 0) {
        temp /= 10;
        numDigits++;
    }
    
    // Display digits
    for (uint8_t i = 0; i < 3; i++) {
        if (i < numDigits) {
            uint8_t digit = speed % 10;
            setSevenSegment(SPEED_START + i, digit, false);
        } else if (displays[SPEED_START + i] != nullptr) {
            displays[SPEED_START + i]->setAllBrightness(0);
        }
        speed /= 10;
    }
}
void DisplayController::displayDriveMode(DriveMode mode) {
    if (mode >= MODE_P && mode <= MODE_S) {
        uint16_t pattern = FOURTEEN_SEG_PATTERNS[mode];
        
        if (displays[MODE_SEGMENT_START] != nullptr) {
            for (uint8_t segment = 0; segment < 7; segment++) {
                bool isOn = pattern & (1 << segment);
                displays[MODE_SEGMENT_START]->setBrightness(segment, isOn ? 50 : 0);
            }
        }
        
        if (displays[MODE_SEGMENT_START + 1] != nullptr) {
            for (uint8_t segment = 0; segment < 7; segment++) {
                bool isOn = pattern & (1 << (segment + 7));
                displays[MODE_SEGMENT_START + 1]->setBrightness(segment, isOn ? 50 : 0);
            }
        }
    }
}

void DisplayController::clear() {
    for (int i = 0; i < NUM_ICS; i++) {
        if (displays[i] != nullptr) {
            displays[i]->setAllBrightness(0);
        }
    }
}

void DisplayController::setSevenSegment(uint8_t icIndex, uint8_t digit, bool showDot) {
    if (icIndex >= NUM_ICS || displays[icIndex] == nullptr) return;
    
    uint8_t pattern = SEVEN_SEG_PATTERNS[digit % 10];
    
    for (uint8_t segment = 0; segment < 7; segment++) {
        uint8_t brightness = !(pattern & (1 << (6 - segment))) ? 50 : 0;
        displays[icIndex]->setBrightness(segment, brightness);
    }
    
    displays[icIndex]->setBrightness(7, showDot ? 50 : 0);
}