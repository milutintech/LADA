// TLC59108.h
#ifndef TLC59108_H
#define TLC59108_H

#include <Arduino.h>
#include <Wire.h>

class TLC59108 {
public:
    // Constants for registers
    enum Register {
        MODE1 = 0x00,
        MODE2 = 0x01,
        PWM0 = 0x02,
        PWM1 = 0x03,
        PWM2 = 0x04,
        PWM3 = 0x05,
        PWM4 = 0x06,
        PWM5 = 0x07,
        PWM6 = 0x08,
        PWM7 = 0x09,
        GRPPWM = 0x0A,
        GRPFREQ = 0x0B,
        LEDOUT0 = 0x0C,
        LEDOUT1 = 0x0D
    };

    // LED output modes
    enum OutputMode {
        OFF = 0x00,
        FULL_ON = 0x01,
        PWM_IND = 0x02,
        PWM_INDGRP = 0x03
    };

    // Constructor
    TLC59108(TwoWire& wire, uint8_t address = 0x40);

    // Initialization
    bool begin();
    
    // Basic control functions
    bool setBrightness(uint8_t channel, uint8_t brightness);
    bool setAllBrightness(uint8_t brightness);
    bool setOutputMode(uint8_t mode);
    
    // Group control
    bool setGroupPWM(uint8_t pwm);
    bool setGroupFrequency(uint8_t freq);

private:
    TwoWire& _wire;
    uint8_t _address;
    
    // Helper functions
    bool writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
};

#endif // TLC59108_H