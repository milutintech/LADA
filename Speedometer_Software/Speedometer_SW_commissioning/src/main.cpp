#include <Arduino.h>
#include <Wire.h>
#include "TLC59108.h"

#define NUM_ICS 15
#define TOTAL_KM_START 0
#define TRIP_KM_START 6
#define SPEED_START 10
#define MODE_SEGMENT_START 13
#define BUTTON_PIN 13
#define DEBOUNCE_TIME 50

const uint8_t SEVEN_SEG_PATTERNS[] = {
    0b10000001,  // 0
    0b11001111,  // 1
    0b00010010,  // 2
    0b00000110,  // 3
    0b01001100,  // 4
    0b00100100,  // 5
    0b10100000,  // 6
    0b10001111,  // 7
    0b10000000,  // 8
    0b10000100   // 9
};

const uint16_t FOURTEEN_SEG_PATTERNS[] = {
    0b10001000110011,  // P
    0b10011000110011,  // R
    0b00010001110110,  // N
    0b00100010001111,  // D
    0b10001000101101   // S
};

class DisplayController;
extern DisplayController* display;

class DisplayController {
public:
    enum DriveMode {
        MODE_P = 0, MODE_R = 1, MODE_N = 2, MODE_D = 3, MODE_S = 4
    };

    DisplayController() {
        displays = new TLC59108*[NUM_ICS];
        Wire.begin(1, 2);
        
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
    
    void displayTotalKm(unsigned long km) {
        km = km; // / 10;  // Convert to display value
        for (uint8_t i = 0; i < 6; i++) {
            uint8_t digit = km % 10;
            setSevenSegment(TOTAL_KM_START + i, digit, false);
            km /= 10;
        }
    }

    void displayTripKm(unsigned long km, uint8_t decimal) {
    // Display decimal digit
    setSevenSegment(TRIP_KM_START + 0, decimal, false);
    
    // Display main digits
    for (uint8_t i = 0; i < 3; i++) {
        uint8_t digit = km % 10;
        setSevenSegment(TRIP_KM_START + (i+1), digit, i == 0);
        km /= 10;
    }
    }
    
    void displaySpeed(unsigned int speed) {
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
    
    void displayDriveMode(DriveMode mode) {
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

private:
    TLC59108** displays;

    void setSevenSegment(uint8_t icIndex, uint8_t digit, bool showDot = false) {
        if (icIndex >= NUM_ICS) return;
        
        uint8_t pattern = SEVEN_SEG_PATTERNS[digit % 10];
        
        for (uint8_t segment = 0; segment < 7; segment++) {
            uint8_t brightness = !(pattern & (1 << (6 - segment))) ? 50 : 0;
            displays[icIndex]->setBrightness(segment, brightness);
        }
        
        displays[icIndex]->setBrightness(7, showDot ? 50 : 0);
    }
};

class DemoMode {
private:
    unsigned long totalKm = 0;
    unsigned long tripKm = 0;
    unsigned int speed = 0;
    int currentDriveMode = 0;
    unsigned long lastUpdate = 0;
    unsigned long lastButtonCheck = 0;
    bool lastButtonState = HIGH;
    unsigned long modeChangeTime = 0;
    uint8_t kmDecimal = 0;

public:
    void setup() {
        pinMode(BUTTON_PIN, INPUT_PULLUP);
    }

    void loop() {
        unsigned long currentTime = millis();

        // Update display every 100ms
        if (currentTime - lastUpdate >= 100) {
            kmDecimal++;
            if (kmDecimal >= 10) {
                kmDecimal = 0;
                totalKm++;
                tripKm++;
            }
            
            speed = random(0, 211);
            
            if (currentTime - modeChangeTime >= 2000) {
                currentDriveMode = (currentDriveMode + 1) % 5;
                modeChangeTime = currentTime;
            }

            display->displayTotalKm(totalKm);
            display->displayTripKm(tripKm, kmDecimal);
            display->displaySpeed(speed);
            display->displayDriveMode(static_cast<DisplayController::DriveMode>(currentDriveMode));
            
            lastUpdate = currentTime;
        }

        // Check button state
        bool currentButtonState = digitalRead(BUTTON_PIN);
        if (currentButtonState == LOW && lastButtonState == HIGH) {
            tripKm = 0;
            kmDecimal = 0;
        }
        lastButtonState = currentButtonState;
    }
};

DisplayController* display;
DemoMode* demoMode;

void setup() {
    Serial.begin(115200);
    delay(1000);
    display = new DisplayController();
    demoMode = new DemoMode();
    demoMode->setup();
}

void loop() {
    demoMode->loop();
}