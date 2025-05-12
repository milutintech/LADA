#ifndef SPEEDOMETER_PINS_H
#define SPEEDOMETER_PINS_H

// Speedometer Pin Definitions
// Based on the ESP32 GPIO numbers (IO pins), not physical pin numbers

// NeoPixel Pin - Keep speedometer's original pin
#define NEOPIXEL_PIN 14  // IO14 for NeoPixel data pin

// CAN and SPI pins - Using IO numbers from the diagram
#define SPI_SCK_PIN 4     // IO4 for SCK
#define SPI_MISO_PIN 5    // IO5 for MISO
#define SPI_MOSI_PIN 6    // IO6 for MOSI
#define CAN_CS_PIN 7      // IO7 for CS
#define CAN_INT_PIN 12    // IO12 for INT (shown as 12 in the diagram)

// I2C pins - Keep speedometer's original pins
#define I2C_SDA_PIN 1     // IO1 for SDA
#define I2C_SCL_PIN 2     // IO2 for SCL

#endif // SPEEDOMETER_PINS_H