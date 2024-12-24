// TLC59108.h
#ifndef TLC59108_H
#define TLC59108_H

#include <Arduino.h>
#include "Wire.h"

class TLC59108 {
public:
    static const byte NUM_CHANNELS = 8;

    // Error codes
    static const uint8_t ERR_OK = 0;
    static const uint8_t ERR_INVALID_PARAM = 2;

    // default I2C addresses
    struct I2C_ADDR {
        static const byte BASE = 0x40;
        static const byte SWRESET = 0x4b;
        static const byte ALLCALL = 0x48;
        static const byte SUB1 = 0x49;
        static const byte SUB2 = 0x4a;
        static const byte SUB3 = 0x4c;
    };

    // register auto-increment modes
    struct AUTO_INCREMENT {
        static const byte ALL = 0x80;
        static const byte IND = 0xa0;
        static const byte GLOBAL = 0xc0;
        static const byte INDGLOBAL = 0xe0;
    };

    struct LED_MODE {
        static const byte OFF = 0;
        static const byte FULL_ON = 1;
        static const byte PWM_IND = 2;
        static const byte PWM_INDGRP = 3;
    };

    // Register addresses
    struct REGISTER {
        struct MODE1 {
            static const byte ADDR = 0x00;
            static const byte OSC_OFF = 0x10;
            static const byte SUB1 = 0x08;
            static const byte SUB2 = 0x04;
            static const byte SUB3 = 0x02;
            static const byte ALLCALL = 0x01;
        };

        struct MODE2 {
            static const byte ADDR = 0x01;
            static const byte EFCLR = 0x80;
            static const byte DMBLNK = 0x20;
            static const byte OCH = 0x08;
        };

        static const byte PWM0 = 0x02;
        static const byte PWM1 = 0x03;
        static const byte PWM2 = 0x04;
        static const byte PWM3 = 0x05;
        static const byte PWM4 = 0x06;
        static const byte PWM5 = 0x07;
        static const byte PWM6 = 0x08;
        static const byte PWM7 = 0x09;
        static const byte GRPPWM = 0x0a;
        static const byte GRPFREQ = 0x0b;
        static const byte LEDOUT0 = 0x0c;
        static const byte LEDOUT1 = 0x0d;
        static const byte SUBADR1 = 0x0e;
        static const byte SUBADR2 = 0x0f;
        static const byte SUBADR3 = 0x10;
        static const byte ALLCALLADR = 0x11;
        static const byte IREF = 0x12;
        static const byte EFLAG = 0x13;
    };

    // Constructors
    TLC59108(TwoWire& i2c, const byte i2c_address);
    explicit TLC59108(const byte i2c_address);

    // Basic operations
    uint8_t init(const uint8_t hwResetPin = 0);
    uint8_t setRegister(const uint8_t reg, const uint8_t value);
    uint8_t setRegisters(const uint8_t startReg, const uint8_t values[], const uint8_t numValues);
    int readRegister(const uint8_t reg) const;
    uint8_t readRegisters(uint8_t *dest, const uint8_t startReg, const uint8_t num) const;

    // LED control
    uint8_t setBrightness(const uint8_t pwmChannel, const uint8_t dutyCycle);
    uint8_t setAllBrightnessSame(const uint8_t dutyCycle);
    uint8_t setAllBrightnessArray(const uint8_t dutyCycles[]);
    bool getAllBrightness(uint8_t dutyCycles[]) const;
    uint8_t setLedOutputMode(const uint8_t outputMode);

private:
    TwoWire& i2c;
    byte addr;
};

#endif // TLC59108_H