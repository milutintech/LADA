// message_scaling.h
#pragma once

class MessageScaling {
protected:
    static uint16_t scaleVoltage(float voltage, float base = 0.0f) {
        return static_cast<uint16_t>((voltage - base) * 10.0f);
    }
    
    static uint16_t scaleCurrent(float current, float offset = 102.4f) {
        return static_cast<uint16_t>((current + offset) * 10.0f);
    }
    
    static float unscaleVoltage(uint16_t value, float base = 0.0f) {
        return (value * 0.1f) + base;
    }
    
    static float unscaleCurrent(uint16_t value, float offset = 102.4f) {
        return (value * 0.1f) - offset;
    }
};
