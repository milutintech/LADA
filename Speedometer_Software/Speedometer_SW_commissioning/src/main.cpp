#include <Arduino.h>
#include <Wire.h>
#include "TLC59108.h"

// Display configuration
#define NUM_ICS 15
#define TOTAL_KM_START 0      // First 6 digits
#define TRIP_KM_START 6       // Next 4 digits
#define SPEED_START 10        // Next 3 digits
#define MODE_SEGMENT_START 13 // Last 2 ICs for 14-segment display

// 7-segment display patterns (0-9) - Inverted for common anode, mirrored layout
const uint8_t SEVEN_SEG_PATTERNS[] = {
    0b10000001,  // 0 (segments: abcdef)
    0b11001111,  // 1 (segments: bc)
    0b00010010,  // 2 (segments: abdeg)
    0b00000110,  // 3 (segments: abcdg)
    0b01001100,  // 4 (segments: bcfg)
    0b00100100,  // 5 (segments: acdfg)
    0b10100000,  // 6 (segments: acdefg)
    0b10001111,  // 7 (segments: abc)
    0b10000000,  // 8 (segments: abcdefg)
    0b10000100   // 9 (segments: abcdfg)
};

// 14-segment patterns for alphanumeric characters PRNDS
const uint16_t FOURTEEN_SEG_PATTERNS[] = {
    0b10001000110011,  // P
    0b10011000110011,  // R
    0b00010001110110,  // N
    0b00100010001111,  // D
    0b10001000101101,  // S
};

// PRNDS

enum DriveMode {
    MODE_P = 0,
    MODE_R = 1,
    MODE_N = 2, 
    MODE_D = 3,
    MODE_S = 4
};

class DisplayController {
private:
    TLC59108** displays;
    
    void setSevenSegment(uint8_t icIndex, uint8_t digit, bool showDot = false) {
        if (icIndex >= NUM_ICS) return;
        
        uint8_t pattern = SEVEN_SEG_PATTERNS[digit % 10];
        
        // Set individual segments (a-g)
        for (uint8_t segment = 0; segment < 7; segment++) {
            uint8_t brightness = !(pattern & (1 << (6 - segment))) ? 50 : 0;
            displays[icIndex]->setBrightness(segment, brightness);
        }
        
        // Set decimal point (DP)
        displays[icIndex]->setBrightness(7, showDot ? 255 : 0);
    }
    
    void setFourteenSegment(uint8_t startIC, char character) {
        if (startIC >= NUM_ICS - 1) return;
        
        uint16_t pattern = FOURTEEN_SEG_PATTERNS[character - 'A'];
        
        // First IC controls segments a-g
        for (uint8_t segment = 0; segment < 7; segment++) {
            bool isOn = pattern & (1 << segment);
            displays[startIC]->setBrightness(segment, isOn ? 50 : 0);
        }
        
        // Second IC controls segments h-n
        for (uint8_t segment = 0; segment < 7; segment++) {
            bool isOn = pattern & (1 << (segment + 7));
            displays[startIC + 1]->setBrightness(segment, isOn ? 50 : 0);
        }
    }

public:
    enum DriveMode {
        MODE_P = 0,
        MODE_R = 1,
        MODE_N = 2, 
        MODE_D = 3,
        MODE_S = 4
    };

    DisplayController() {
        displays = new TLC59108*[NUM_ICS];
        
        // Initialize I2C
        Wire.begin(1, 2);  // SDA = 1, SCL = 2
        
        // Initialize all TLC59108 instances
        for (int i = 0; i < NUM_ICS; i++) {
            byte address = 0x40 + i;  // Base address 0x40, increment for each IC
            displays[i] = new TLC59108(Wire, address);
            displays[i]->begin();  // Initialize chip
            displays[i]->setOutputMode(2);  // PWM Individual mode
            for (int j = 0; j < 8; j++) {
                displays[i]->setBrightness(j, 0);  // Turn all segments off initially
            }
        }
    }
    
    void displayTotalKm(unsigned long km) {
        for (uint8_t i = 0; i < 6; i++) {
            uint8_t digit = km % 10;
            setSevenSegment(TOTAL_KM_START + i, digit, false);  // Changed (5-i) to i
            km /= 10;
        }
    }
    
    void displayTripKm(unsigned long km) {
        unsigned long tripValue = km;
        for (uint8_t i = 0; i < 4; i++) {
            uint8_t digit = tripValue % 10;
            setSevenSegment(TRIP_KM_START + i, digit, i == 1);  // Decimal point after first digit
            tripValue /= 10;
        }
    }
    
    void displaySpeed(unsigned int speed) {
        for (uint8_t i = 0; i < 3; i++) {
            uint8_t digit = speed % 10;
            setSevenSegment(SPEED_START + i, digit);
            speed /= 10;
        }
    }
    
    void displayDriveMode(DriveMode mode) {
        if (mode >= MODE_P && mode <= MODE_S) {
            uint16_t pattern = FOURTEEN_SEG_PATTERNS[mode];
            
            // First IC controls segments a-g
            for (uint8_t segment = 0; segment < 7; segment++) {
                bool isOn = pattern & (1 << segment);
                displays[MODE_SEGMENT_START]->setBrightness(segment, isOn ? 255 : 0);
            }
            
            // Second IC controls segments h-n
            for (uint8_t segment = 0; segment < 7; segment++) {
                bool isOn = pattern & (1 << (segment + 7));
                displays[MODE_SEGMENT_START + 1]->setBrightness(segment, isOn ? 255 : 0);
            }
        }
    }
    
    void clear() {
        for (int i = 0; i < NUM_ICS; i++) {
            displays[i]->setAllBrightness(0);
        }
    }
    
    ~DisplayController() {
        for (int i = 0; i < NUM_ICS; i++) {
            delete displays[i];
        }
        delete[] displays;
    }
};

DisplayController* display;

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Initializing Display System");
    
    display = new DisplayController();
    
    // Test all display components
    display->displayTotalKm(777777);
    display->displayTripKm(1234);
    display->displaySpeed(85);
    display->displayDriveMode(DisplayController::MODE_N);
}

void loop() {
    // Add your display update logic here
    delay(100);
}