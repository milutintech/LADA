#include "DisplayController.h"
#include "Globals.h"

extern const uint8_t SEVEN_SEG_PATTERNS[];
extern const uint16_t FOURTEEN_SEG_PATTERNS[];

DisplayController::DisplayController() {
    displays = new TLC59108*[NUM_ICS];
    icAddresses = new byte[NUM_ICS];
    validDisplays = new bool[NUM_ICS];

    // Initialize I2C Bus 1 addresses (indices 0-9: Total KM + Trip KM)
    // All use sequential addresses 0x40-0x49
    for (int i = BUS1_START; i <= BUS1_END; i++) {
        icAddresses[i] = 0x40 + i;
        validDisplays[i] = true;
    }

    // Initialize I2C Bus 2 addresses (indices 10-14: Speed + Drive Mode)
    // Use addresses 0x40-0x44 on the second bus
    for (int i = BUS2_START; i <= BUS2_END; i++) {
        icAddresses[i] = 0x40 + (i - BUS2_START);  // Maps 10→0x40, 11→0x41, 12→0x42, 13→0x43, 14→0x44
        validDisplays[i] = true;
    }

    // Initialize all displays with their corresponding I2C bus and addresses
    for (int i = 0; i < NUM_ICS; i++) {
        if (!validDisplays[i]) {
            DEBUG_PRINTF("Skipping display at index %d (marked as removed)\n", i);
            displays[i] = nullptr;
            continue;
        }

        // Select the appropriate I2C bus
        TwoWire* i2cBus;
        const char* busName;
        if (i >= BUS1_START && i <= BUS1_END) {
            i2cBus = &Wire;    // I2C Bus 1
            busName = "Bus1";
        } else if (i >= BUS2_START && i <= BUS2_END) {
            i2cBus = &Wire1;   // I2C Bus 2
            busName = "Bus2";
        } else {
            Serial.printf("Invalid display index %d\n", i);
            displays[i] = nullptr;
            continue;
        }

        displays[i] = new TLC59108(*i2cBus, icAddresses[i]);

        // Begin communication with the display
        if (!displays[i]->begin()) {
            DEBUG_PRINTF("Failed to initialize display at address 0x%02X (index %d, %s)\n",
                         icAddresses[i], i, busName);
        } else {
            DEBUG_PRINTF("Successfully initialized display at address 0x%02X (index %d, %s)\n",
                         icAddresses[i], i, busName);
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
    // Display all 6 digits (1 km to 100,000 km)
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
    // Special case for 0 - Fixed to avoid flickering
    if (speed == 0) {
        // Always display a zero in the ones place rather than turning all off
        setSevenSegment(SPEED_START, 0, false);

        // Turn off tens and hundreds digits with error checking
        if (displays[SPEED_START + 1] != nullptr && validDisplays[SPEED_START + 1]) {
            if (!displays[SPEED_START + 1]->setAllBrightness(0)) {
                validDisplays[SPEED_START + 1] = false;  // Mark as invalid on error
                VERBOSE_PRINTF("Speed tens digit (0x%02X) not responding\n", icAddresses[SPEED_START + 1]);
            }
        }
        if (displays[SPEED_START + 2] != nullptr && validDisplays[SPEED_START + 2]) {
            if (!displays[SPEED_START + 2]->setAllBrightness(0)) {
                validDisplays[SPEED_START + 2] = false;  // Mark as invalid on error
                VERBOSE_PRINTF("Speed hundreds digit (0x%02X) not responding\n", icAddresses[SPEED_START + 2]);
            }
        }
        return;
    }

    unsigned int temp = speed;
    int numDigits = 0;

    // Count number of digits
    while (temp > 0) {
        temp /= 10;
        numDigits++;
    }

    // Display digits with error checking
    for (uint8_t i = 0; i < 3; i++) {
        if (displays[SPEED_START + i] != nullptr && validDisplays[SPEED_START + i]) {
            if (i < numDigits) {
                uint8_t digit = speed % 10;
                setSevenSegment(SPEED_START + i, digit, false);
            } else {
                if (!displays[SPEED_START + i]->setAllBrightness(0)) {
                    validDisplays[SPEED_START + i] = false;  // Mark as invalid on error
                    VERBOSE_PRINTF("Speed display %d (0x%02X) not responding\n", i, icAddresses[SPEED_START + i]);
                }
            }
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
                displays[MODE_SEGMENT_START]->setBrightness(segment, isOn ? brightnessDriveMode : 0);
            }
        }

        if (displays[MODE_SEGMENT_START + 1] != nullptr) {
            for (uint8_t segment = 0; segment < 7; segment++) {
                bool isOn = pattern & (1 << (segment + 7));
                displays[MODE_SEGMENT_START + 1]->setBrightness(segment, isOn ? brightnessDriveMode : 0);
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
    if (!validDisplays[icIndex]) return;  // Skip invalid displays

    uint8_t pattern = SEVEN_SEG_PATTERNS[digit % 10];

    for (uint8_t segment = 0; segment < 7; segment++) {
        uint8_t brightness = !(pattern & (1 << (6 - segment))) ? brightness7Segment : 0;
        if (!displays[icIndex]->setBrightness(segment, brightness)) {
            validDisplays[icIndex] = false;  // Mark as invalid on error
            VERBOSE_PRINTF("Display %d (0x%02X) failed during segment write\n", icIndex, icAddresses[icIndex]);
            return;
        }
    }

    if (!displays[icIndex]->setBrightness(7, showDot ? brightness7Segment : 0)) {
        validDisplays[icIndex] = false;
        VERBOSE_PRINTF("Display %d (0x%02X) failed during dot write\n", icIndex, icAddresses[icIndex]);
    }

    // Small delay after each digit update to reduce I2C bus congestion
    delayMicroseconds(50);
}