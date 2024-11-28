#include <Arduino.h>
#include <esp_task_wdt.h>
#include <Wire.h>
#include <SPI.h>
#include "ADS1X15.h"
#include "AD5593R.h"
#include "mode_manager.h"
#include "bsc_handler.h"
#include "dmc_handler.h"
#include "nlg_handler.h"
#include "bms_handler.h"

// Global objects
SPIClass *customSPI = nullptr;
mcp2515_can CAN(SPI_CS_PIN);
ADS1115 ADS(0x48);
ModeManager modeManager;
BSCHandler bscHandler(CAN);
DMCHandler dmcHandler(CAN);
NLGHandler nlgHandler(CAN);
BMSHandler bmsHandler(CAN);

// Task handles
TaskHandle_t Task1;
TaskHandle_t Task2;

// Interrupt handler for connector unlock
void IRAM_ATTR unlockCON() {
    if(nlgHandler.isConnectorLocked()) {
        modeManager.changeMode(VehicleMode::Charging);
        conUlockInterrupt = true;
    } else {
        modeManager.changeMode(VehicleMode::Standby);
    }
}

void initializeHardware() {
    // Initialize SPI
    customSPI = new SPIClass(HSPI);
    customSPI->begin(SCK, MISO, MOSI, SPI_CS_PIN);
    CAN.setSPI(customSPI);

    // Initialize I2C and ADC
    Wire.begin(1, 2);
    ADS.begin();
    ADS.setGain(2);
    initADAC(0b1001000, 1, 1);
    setADCpin(0);

    // Initialize GPIO
    pinMode(IGNITION, INPUT);
    pinMode(NLG_HW_Wakeup, INPUT);
    pinMode(UNLCKCON, INPUT);
    pinMode(CONTACTOR, OUTPUT);
    pinMode(PUMP, OUTPUT);
    pinMode(NLGKL15, OUTPUT);
    pinMode(DMCKL15, OUTPUT);
    pinMode(BSCKL15, OUTPUT);
    pinMode(BCKLIGHT, OUTPUT);

    // Setup interrupts
    esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH);
    attachInterrupt(digitalPinToInterrupt(UNLCKCON), unlockCON, RISING);
}

void CAN_COM(void* pvParameters) {
    Serial.begin(115200);
    
    while (CAN_OK != CAN.begin(CAN_500KBPS)) {
        Serial.println("CAN init fail");
        delay(100);
    }
    
    while(true) {
        esp_task_wdt_init(5, true);
        bmsHandler.receiveBMS();
        modeManager.handleCommunication();
        delay(1); // Prevent watchdog timeout
    }
}

void BACKBONE(void* pvParameters) {
    while(true) {
        esp_task_wdt_init(5, true);
        modeManager.handleCurrentMode();
        delay(1); // Prevent watchdog timeout
    }
}

void setup() {
    initializeHardware();
    
    // Create tasks
    xTaskCreatePinnedToCore(
        CAN_COM,
        "Task1",
        10000,
        NULL,
        1,
        &Task1,
        0
    );
    
    xTaskCreatePinnedToCore(
        BACKBONE,
        "Task2",
        20000,
        NULL,
        1,
        &Task2,
        1
    );
}

void loop() {
    // Empty - tasks handle main functionality
}