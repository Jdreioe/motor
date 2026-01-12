#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include "sensorDriver.h"
#include "led.h"
#include "motorController.h"

// State variables
static uint8_t last_left = 0;
static uint8_t last_right = 0;
static uint16_t reflex_count = 0;

// Timing variables for non-blocking delay
static uint8_t led_active = 0;
static uint16_t led_start_ticks = 0;

void sensorInit(void) {
    // PD2 og PD3 som input (sensorer)
    DDRD &= ~((1 << PD2) | (1 << PD3));
    // Intern pull-up på PD2 og PD3
    PORTD |= (1 << PD2) | (1 << PD3);
    
    // LED initialization is handled by initLEDport() in main
}

void resetReflexCount(void) {
    reflex_count = 0;
}

void sensorUpdate(void) {
    uint16_t current_ticks = motorGetTicks();

    // Handle LED timing if active
    if (led_active) {
        uint16_t elapsed = (uint16_t)(current_ticks - led_start_ticks);
        
        // Turn off LED after 500ms (100 ticks @ 200Hz)
        if (elapsed >= 100) {
            turnOffLED(0);
        }
        
        // After ignore period (500ms + 5ms = 101 ticks), reset to allow new detections
        if (elapsed >= 101) {
            led_active = 0;
            // Update last sensor state to current state
            last_left = (PIND & (1 << PD2)) ? 1 : 0;
            last_right = (PIND & (1 << PD3)) ? 1 : 0;
        }
        return; // Don't check for new reflexes while LED is active or ignoring
    }

    // Read sensors
    uint8_t left = (PIND & (1 << PD2)) ? 1 : 0;
    uint8_t right = (PIND & (1 << PD3)) ? 1 : 0;

    // Check for new reflex (edge detection)
    if ((left == 1 && last_left == 0) || (right == 1 && last_right == 0)) {
        reflex_count++;
        turnOnLED(0);
        led_start_ticks = current_ticks;
        led_active = 1;
    }
    
    // Save current sensor state
    last_left = left;
    last_right = right;
}

uint16_t getReflexCount(void) {
    return reflex_count;
}
