// TLC59108.cpp
#include "TLC59108.h"

TLC59108::TLC59108(TwoWire& i2c, const byte i2c_address) 
    : i2c(i2c), addr(i2c_address) {
}

TLC59108::TLC59108(const byte i2c_address) 
    : i2c(Wire), addr(i2c_address) {
}

uint8_t TLC59108::init(const uint8_t hwResetPin) {
    if(hwResetPin) {
        pinMode(hwResetPin, OUTPUT);
        digitalWrite(hwResetPin, LOW);
        delay(1);
        digitalWrite(hwResetPin, HIGH);
        delay(1);
    }

    return setRegister(REGISTER::MODE1::ADDR, REGISTER::MODE1::ALLCALL);
}

uint8_t TLC59108::setRegister(const uint8_t reg, const uint8_t value) {
    i2c.beginTransmission(addr);
    i2c.write(reg);
    i2c.write(value);
    return i2c.endTransmission();
}

uint8_t TLC59108::setRegisters(const uint8_t startReg, const uint8_t values[], const uint8_t numValues) {
    i2c.beginTransmission(addr);
    i2c.write(startReg | AUTO_INCREMENT::ALL);
    for(uint8_t i = 0; i < numValues; i++)
        i2c.write(values[i]);
    return i2c.endTransmission();
}

int TLC59108::readRegister(const uint8_t reg) const {
    i2c.beginTransmission(addr);
    i2c.write(reg);
    if(i2c.endTransmission() != 0)
        return -1;

    i2c.requestFrom(addr, (uint8_t) 1);
    if(i2c.available())
        return i2c.read();
    else
        return -1;
}

uint8_t TLC59108::readRegisters(uint8_t *dest, const uint8_t startReg, const uint8_t num) const {
    i2c.beginTransmission(addr);
    i2c.write(startReg | AUTO_INCREMENT::ALL);
    if(i2c.endTransmission())
        return 0;

    uint8_t bytesRead = 0;
    i2c.requestFrom(addr, num);
    while(i2c.available() && (bytesRead < num)) {
        (*dest) = (uint8_t) i2c.read();
        dest++;
        bytesRead++;
    }

    return bytesRead;
}

uint8_t TLC59108::setBrightness(const uint8_t pwmChannel, const uint8_t dutyCycle) {
    if(pwmChannel > 7) {
        return ERR_INVALID_PARAM;
    }

    return setRegister(REGISTER::PWM0 + pwmChannel, dutyCycle);
}

uint8_t TLC59108::setAllBrightnessSame(const uint8_t dutyCycle) {
    i2c.beginTransmission(addr);
    i2c.write(REGISTER::PWM0 | AUTO_INCREMENT::IND);
    for(uint8_t i = 0; i < NUM_CHANNELS; i++)
        i2c.write(dutyCycle);
    return i2c.endTransmission();
}

uint8_t TLC59108::setAllBrightnessArray(const uint8_t dutyCycles[]) {
    return setRegisters(REGISTER::PWM0, dutyCycles, NUM_CHANNELS);
}

bool TLC59108::getAllBrightness(uint8_t dutyCycles[]) const {
    return (readRegisters(dutyCycles, REGISTER::PWM0, NUM_CHANNELS) == NUM_CHANNELS);
}

uint8_t TLC59108::setLedOutputMode(const uint8_t outputMode) {
    if(outputMode & 0xfc) {
        return ERR_INVALID_PARAM;
    }

    byte regValue = (outputMode << 6) | (outputMode << 4) | (outputMode << 2) | outputMode;

    uint8_t retVal = setRegister(REGISTER::LEDOUT0, regValue);
    retVal &= setRegister(REGISTER::LEDOUT1, regValue);
    return retVal;
}