// TLC59108.cpp
#include "TLC59108.h"

TLC59108::TLC59108(TwoWire& wire, uint8_t address) : _wire(wire), _address(address) {
}

bool TLC59108::begin() {
    // Initialize with default settings
    // Set MODE1 to normal operation
    if (!writeRegister(MODE1, 0x00)) return false;
    
    // Set MODE2 to push-pull outputs, outputs change on STOP
    if (!writeRegister(MODE2, 0x00)) return false;
    
    // Set all LEDs to PWM control mode
    if (!writeRegister(LEDOUT0, 0xAA)) return false;  // PWM mode for channels 0-3
    if (!writeRegister(LEDOUT1, 0xAA)) return false;  // PWM mode for channels 4-7
    
    return true;
}

bool TLC59108::setBrightness(uint8_t channel, uint8_t brightness) {
    if (channel > 7) return false;
    return writeRegister(PWM0 + channel, brightness);
}

bool TLC59108::setAllBrightness(uint8_t brightness) {
    for (uint8_t i = 0; i < 8; i++) {
        if (!setBrightness(i, brightness)) return false;
    }
    return true;
}

bool TLC59108::setOutputMode(uint8_t mode) {
    uint8_t ledout = 0;
    for (uint8_t i = 0; i < 4; i++) {
        ledout |= (mode & 0x03) << (i * 2);
    }
    
    if (!writeRegister(LEDOUT0, ledout)) return false;
    if (!writeRegister(LEDOUT1, ledout)) return false;
    
    return true;
}

bool TLC59108::setGroupPWM(uint8_t pwm) {
    return writeRegister(GRPPWM, pwm);
}

bool TLC59108::setGroupFrequency(uint8_t freq) {
    return writeRegister(GRPFREQ, freq);
}

bool TLC59108::writeRegister(uint8_t reg, uint8_t value) {
    _wire.beginTransmission(_address);
    _wire.write(reg);
    _wire.write(value);
    return (_wire.endTransmission() == 0);
}

uint8_t TLC59108::readRegister(uint8_t reg) {
    _wire.beginTransmission(_address);
    _wire.write(reg);
    if (_wire.endTransmission(false) != 0) return 0;
    
    _wire.requestFrom(_address, (uint8_t)1);
    if (_wire.available()) {
        return _wire.read();
    }
    return 0;
}