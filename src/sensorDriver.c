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

// State machine for non-blocking delay
typedef enum {
    SENSOR_STATE_IDLE,
    SENSOR_STATE_LED_ON,
    SENSOR_STATE_IGNORE
} SensorState;

static SensorState current_state = SENSOR_STATE_IDLE;
static uint16_t state_start_ticks = 0;

void sensorInit(void) {
    // PD2 og PD3 som input (sensorer)
    DDRD &= ~((1 << PD2) | (1 << PD3));
    // Intern pull-up på PD2 og PD3
    PORTD |= (1 << PD2) | (1 << PD3);
    
    // LED initialization is handled by initLEDport() in main
}

void sensorUpdate(void) {
    uint16_t current_ticks = motorGetTicks();

    switch (current_state) {
        case SENSOR_STATE_IDLE: {
            // Læs sensorer
            uint8_t left = (PIND & (1 << PD2)) ? 1 : 0;
            uint8_t right = (PIND & (1 << PD3)) ? 1 : 0;

            // Tjek om der er en "ny" refleks:
            if ( (left == 1 && last_left == 0) ||
                 (right == 1 && last_right == 0) )
            {
                // Vi har passeret én refleks
                reflex_count++;

                // Tænd LED
                turnOnLED(0);
                
                // Start timer for LED ON duration (2000ms = 400 ticks @ 200Hz)
                state_start_ticks = current_ticks;
                current_state = SENSOR_STATE_LED_ON;
            }
            
            // Gem nuværende tilstand til næste loop
            last_left = left;
            last_right = right;
            break;
        }

        case SENSOR_STATE_LED_ON: {
            // Wait for 2000ms (400 ticks)
            if ((uint16_t)(current_ticks - state_start_ticks) >= 100) {
                turnOffLED(0);
                
                // Start timer for IGNORE duration (5ms = 1 tick @ 200Hz)
                state_start_ticks = current_ticks;
                current_state = SENSOR_STATE_IGNORE;
            }
            break;
        }

        case SENSOR_STATE_IGNORE: {
            // Wait for 5ms (1 tick)
            if ((uint16_t)(current_ticks - state_start_ticks) >= 1) {
                // Read sensors to update last state before going back to IDLE
                last_left = (PIND & (1 << PD2)) ? 1 : 0;
                last_right = (PIND & (1 << PD3)) ? 1 : 0;
                
                current_state = SENSOR_STATE_IDLE;
            }
            break;
        }
    }
}

uint16_t getReflexCount(void) {
    return reflex_count;
}
