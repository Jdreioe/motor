#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include "sensorDriver.h"

// State variables
static uint8_t last_left = 0;
static uint8_t last_right = 0;
static uint16_t reflex_count = 0;

void sensorInit(void) {
    // PD2 og PD3 som input (sensorer)
    DDRD &= ~((1 << PD2) | (1 << PD3));
    // Intern pull-up på PD2 og PD3
    PORTD |= (1 << PD2) | (1 << PD3);
    
    // PB0 som output (LED der blinker ved refleks)
    DDRB |= (1 << PB0);
}

void sensorUpdate(void) {
    // Læs sensorer
    uint8_t left = (PIND & (1 << PD2)) ? 1 : 0;
    uint8_t right = (PIND & (1 << PD3)) ? 1 : 0;

    // Tjek om der er en "ny" refleks:
    // venstre: 0 -> 1
    // højre:   0 -> 1
    if ( (left == 1 && last_left == 0) ||
         (right == 1 && last_right == 0) )
    {
        // Vi har passeret én refleks
        reflex_count++;

        // Blink LED for at vise registrering - SKAL TESTES!
        PORTB |= (1 << PB0);
        _delay_ms(50);
        PORTB &= ~(1 << PB0);

        // Ignorer begge sensorer i noget tid så vi ikke dobbeltregistrerer. Delay skal TESTES!
        _delay_ms(300);

        // Læs sensorer igen efter bilen er kørt lidt videre - nulstiller refleks
        left = (PIND & (1 << PD2)) ? 1 : 0;
        right = (PIND & (1 << PD3)) ? 1 : 0;
    }

    // Gem nuværende tilstand til næste loop
    last_left = left;
    last_right = right;
}

uint16_t getReflexCount(void) {
    return reflex_count;
}
