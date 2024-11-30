#include "setup.h"
#include "config.h"
#include <Wire.h>
#include <SPI.h>
#include <driver/adc.h>
#include <esp_task_wdt.h>
#include <esp_sleep.h>
#include <ADS1X15.h>

// Helper function declarations should be inside class definition

void SystemSetup::initializeSystem(
    ADS1115& ads,
    CANManager& canManager,
    StateManager& stateManager,
    VehicleControl& vehicleControl,
    TaskHandle_t& canTaskHandle,
    TaskHandle_t& controlTaskHandle
) {
    // Initialize GPIO pins
    initializeGPIO();
    
    // Initialize I2C for ADC
    initializeI2C();
    
    // Initialize ADC
    initializeADC(ads);
    
    // Initialize SPI for CAN
    initializeSPI();
    initializeUART();
    // Initialize CAN
    canManager.begin();
    
    // Setup sleep configuration
    initializeSleep();
    
    // Setup watchdog
    initializeWatchdog();
    
    // Setup interrupt handlers
    setupInterrupts();
    
    // Perform initial safety checks
    performInitialSafetyChecks();
}

void SystemSetup::initializeGPIO() {
    // Configure input pins
    pinMode(Pins::NLG_HW_Wakeup, INPUT);
    pinMode(Pins::IGNITION, INPUT);
    pinMode(Pins::UNLCKCON, INPUT);
    
    // Configure output pins
    pinMode(Pins::PUMP, OUTPUT);
    pinMode(Pins::CONTACTOR, OUTPUT);
    pinMode(Pins::NLGKL15, OUTPUT);
    pinMode(Pins::DMCKL15, OUTPUT);
    pinMode(Pins::BSCKL15, OUTPUT);
    pinMode(Pins::BCKLIGHT, OUTPUT);
    
    // Set default pin states
    setDefaultPinStates();
}

void SystemSetup::initializeI2C() {
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock
}

void SystemSetup::initializeADC(ADS1115& ads) {
    ads.begin();
    ads.setGain(2);    // Set the ADC gain to ±4.096V
}

void SystemSetup::initializeSPI() {
    SPI.begin(Pins::SCK, Pins::MISO, Pins::MOSI, Pins::SPI_CS_PIN);
}
void SystemSetup::initializeUART() {
    Serial.begin(115200);
    while(!Serial) {
        ; // Wait for Serial to be ready
    }
    Serial.println("\nVCU System Starting...");
}
void SystemSetup::initializeSleep() {
    esp_sleep_enable_ext0_wakeup(static_cast<gpio_num_t>(Pins::IGNITION), 1);
    esp_sleep_enable_ext1_wakeup(Pins::BUTTON_PIN_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH);
}

void SystemSetup::initializeWatchdog() {
    #ifdef CONFIG_ESP_TASK_WDT_EN
    esp_task_wdt_init(5, true);  // 5 second timeout
    #endif
}

void SystemSetup::setupInterrupts() {
    // Add any required interrupt handlers here
}

void SystemSetup::setDefaultPinStates() {
    digitalWrite(Pins::PUMP, LOW);
    digitalWrite(Pins::CONTACTOR, LOW);
    digitalWrite(Pins::NLGKL15, LOW);
    digitalWrite(Pins::DMCKL15, LOW);
    digitalWrite(Pins::BSCKL15, LOW);
    digitalWrite(Pins::BCKLIGHT, LOW);
}

void SystemSetup::performInitialSafetyChecks() {
    // Add any required safety checks here
}