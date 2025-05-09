#include "DisplayController.h"

extern const uint8_t SEVEN_SEG_PATTERNS[];
extern const uint16_t FOURTEEN_SEG_PATTERNS[];

DisplayController::DisplayController() {
    displays = new TLC59108*[NUM_ICS];
    
    for (int i = 0; i < NUM_ICS; i++) {
        byte address = 0x40 + i;
        displays[i] = new TLC59108(Wire, address);
        displays[i]->begin();
        displays[i]->setOutputMode(2);
        for (int j = 0; j < 8; j++) {
            displays[i]->setBrightness(j, 0);
        }
    }
}

DisplayController::~DisplayController() {
    for (int i = 0; i < NUM_ICS; i++) {
        delete displays[i];
    }
    delete[] displays;
}

void DisplayController::displayTotalKm(unsigned long km) {
    for (uint8_t i = 0; i < 6; i++) {
        uint8_t digit = km % 10;
        setSevenSegment(TOTAL_KM_START + i, digit, false);
        km /= 10;
    }
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
    // Special case for 0
    if (speed == 0) {
        displays[SPEED_START + 1]->setAllBrightness(0);
        displays[SPEED_START + 2]->setAllBrightness(0);
        setSevenSegment(SPEED_START, 0, false);
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
        } else {
            displays[SPEED_START + i]->setAllBrightness(0);
        }
        speed /= 10;
    }
}

void DisplayController::displayDriveMode(DriveMode mode) {
    if (mode >= MODE_P && mode <= MODE_S) {
        uint16_t pattern = FOURTEEN_SEG_PATTERNS[mode];
        
        for (uint8_t segment = 0; segment < 7; segment++) {
            bool isOn = pattern & (1 << segment);
            displays[MODE_SEGMENT_START]->setBrightness(segment, isOn ? 50 : 0);
        }
        
        for (uint8_t segment = 0; segment < 7; segment++) {
            bool isOn = pattern & (1 << (segment + 7));
            displays[MODE_SEGMENT_START + 1]->setBrightness(segment, isOn ? 50 : 0);
        }
    }
}

void DisplayController::clear() {
    for (int i = 0; i < NUM_ICS; i++) {
        displays[i]->setAllBrightness(0);
    }
}

void DisplayController::setSevenSegment(uint8_t icIndex, uint8_t digit, bool showDot) {
    if (icIndex >= NUM_ICS) return;
    
    uint8_t pattern = SEVEN_SEG_PATTERNS[digit % 10];
    
    for (uint8_t segment = 0; segment < 7; segment++) {
        uint8_t brightness = !(pattern & (1 << (6 - segment))) ? 50 : 0;
        displays[icIndex]->setBrightness(segment, brightness);
    }
    
    displays[icIndex]->setBrightness(7, showDot ? 50 : 0);
}