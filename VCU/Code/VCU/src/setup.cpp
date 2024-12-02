#include "setup.h"
#include "config.h"
#include <esp_sleep.h>

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
    pinMode(Pins::PW1, OUTPUT);
    pinMode(Pins::LWP5, OUTPUT);
    pinMode(Pins::LWP6, OUTPUT);
    pinMode(Pins::LWP7, OUTPUT);
    
    setDefaultPinStates();
}

void SystemSetup::initializeSleep() {
    esp_sleep_enable_ext0_wakeup(static_cast<gpio_num_t>(Pins::IGNITION), 1);
    esp_sleep_enable_ext1_wakeup(Pins::BUTTON_PIN_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH);
}

void SystemSetup::setDefaultPinStates() {
    digitalWrite(Pins::PUMP, LOW);
    digitalWrite(Pins::CONTACTOR, LOW);
    digitalWrite(Pins::NLGKL15, LOW);
    digitalWrite(Pins::DMCKL15, LOW);
    digitalWrite(Pins::BSCKL15, LOW);
    digitalWrite(Pins::BCKLIGHT, LOW);
    digitalWrite(Pins::PW1, LOW);
    digitalWrite(Pins::LWP5, LOW);
    digitalWrite(Pins::LWP6, LOW);
    digitalWrite(Pins::LWP7, LOW);
}