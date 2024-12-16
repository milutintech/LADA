// src/main.cpp
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN 14
#define LED_COUNT 69
#define GROUP_SIZE 5  // Number of LEDs to light up at once
#define DELAY_MS 100  // Delay between animation steps

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// Colors
uint32_t colors[] = {
    strip.Color(255, 0, 0),     // Red
    strip.Color(0, 255, 0),     // Green
    strip.Color(0, 0, 255),     // Blue
    strip.Color(255, 255, 0),   // Yellow
    strip.Color(0, 255, 255)    // Cyan
};
const int NUM_COLORS = sizeof(colors) / sizeof(colors[0]);

void setup() {
    strip.begin();
    strip.setBrightness(50);  // Set to 50% brightness to start
    strip.show();  // Initialize all pixels to 'off'
}

void loop() {
    // Pattern 1: Moving group of lit LEDs
    for (int startPos = 0; startPos < LED_COUNT; startPos++) {
        strip.clear();
        
        // Light up GROUP_SIZE LEDs at current position
        for (int i = 0; i < GROUP_SIZE; i++) {
            int pos = (startPos + i) % LED_COUNT;
            strip.setPixelColor(pos, colors[startPos % NUM_COLORS]);
        }
        
        strip.show();
        delay(DELAY_MS);
    }
    
    // Pattern 2: Random groups of LEDs
    for (int i = 0; i < 20; i++) {  // Do 20 random groups
        strip.clear();
        
        int startPos = random(0, LED_COUNT);
        uint32_t color = colors[random(0, NUM_COLORS)];
        
        for (int j = 0; j < GROUP_SIZE; j++) {
            int pos = (startPos + j) % LED_COUNT;
            strip.setPixelColor(pos, color);
        }
        
        strip.show();
        delay(DELAY_MS * 2);
    }
    
    // Pattern 3: All LEDs same color, changing colors
    for (int colorIndex = 0; colorIndex < NUM_COLORS; colorIndex++) {
        strip.fill(colors[colorIndex]);
        strip.show();
        delay(500);
    }
}