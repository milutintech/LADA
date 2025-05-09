#ifndef SPEEDOMETER_CONTROLLER_H
#define SPEEDOMETER_CONTROLLER_H

#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include <Adafruit_NeoPixel.h>

// Pin Definitions
#define NEOPIXEL_PIN 14  // NeoPixel data pin
#define CAN_CS_PIN 7
#define CAN_INT_PIN 3   // Optional interrupt pin

// NeoPixel section sizes
#define TORQUE_PIXELS 20       // 0-19: torque display
#define SOC_PIXELS 8          // 20-27: state of charge
#define SOC_MARKER_PIXELS 9    // 28-36: SOC markers
#define ERROR_PIXELS 13        // 37-49: error lights
#define TEMP_PIXELS 8         // 50-57: temperature
#define TEMP_MARKER_PIXELS 10  // 58-67: temperature markers
#define ILLUMINATION_PIXELS 30 // 68-97: illumination
#define TOTAL_PIXELS (TORQUE_PIXELS + SOC_PIXELS + SOC_MARKER_PIXELS + ERROR_PIXELS + TEMP_PIXELS + TEMP_MARKER_PIXELS + ILLUMINATION_PIXELS)

// Debug prints
#define DEBUG_PRINTS 1

class SpeedometerController {
public:
    SpeedometerController();
    bool begin();
    void updateTorque(int16_t torque);
    void updateSOC(uint8_t percentage);
    void updateErrorLights(uint16_t errorFlags);
    void updateTemperature(uint8_t temp);
    void setIllumination(uint8_t brightness);
    void processCANMessages();
    void show();

private:
    Adafruit_NeoPixel pixels;
    MCP2515 can;

    void clearPixelRange(int start, int count);
    void setPixelRange(int start, int end, uint32_t color);
    void debugPrint(String msg);
};

#endif // SPEEDOMETER_CONTROLLER_H