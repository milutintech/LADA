#ifndef GLOBALS_H
#define GLOBALS_H

// Verbose output levels
#define VERBOSE_SILENT 0      // Errors only
#define VERBOSE_NORMAL 1      // Important events (mode changes, errors, warnings)
#define VERBOSE_DEBUG 2       // All messages (init + runtime updates)

// Global verbose level
extern uint8_t verboseLevel;

// Display brightness settings (0-255 for 7-segment, 0-255 for NeoPixels)
extern uint8_t brightness7Segment;        // Speed, Total KM, Trip KM displays
extern uint8_t brightnessDriveMode;       // PNDRS display
extern uint8_t brightnessNeoPixelGlobal;  // Global NeoPixel brightness
extern uint8_t brightnessMarkers;         // SOC and Temp marker LEDs (multiplier, 0-255)
extern uint8_t brightnessDataBars;        // Torque, SOC, Temp data bars (multiplier, 0-255)
extern uint8_t brightnessErrorLights;     // Error/status indicator lights (multiplier, 0-255)

// Helper macros for conditional printing
// VERBOSE_NORMAL: Important messages (always show unless silent)
#define VERBOSE_PRINT(x) if (verboseLevel >= VERBOSE_NORMAL) Serial.print(x)
#define VERBOSE_PRINTLN(x) if (verboseLevel >= VERBOSE_NORMAL) Serial.println(x)
#define VERBOSE_PRINTF(...) if (verboseLevel >= VERBOSE_NORMAL) Serial.printf(__VA_ARGS__)

// VERBOSE_DEBUG: Debug/diagnostic messages (only in debug mode)
#define DEBUG_PRINT(x) if (verboseLevel >= VERBOSE_DEBUG) Serial.print(x)
#define DEBUG_PRINTLN(x) if (verboseLevel >= VERBOSE_DEBUG) Serial.println(x)
#define DEBUG_PRINTF(...) if (verboseLevel >= VERBOSE_DEBUG) Serial.printf(__VA_ARGS__)

#endif // GLOBALS_H
