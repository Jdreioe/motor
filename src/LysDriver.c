#include "lysdriver.h"
#include <avr/io.h>
#include <avr/interrupt.h>

// Software PWM state for two channels: headlight (PB4) and rear/brake (PH6)
static volatile uint8_t pwm_counter = 0; // shared 0..255 counter
static volatile uint8_t head_pwm_duty = 0; // PB4
static volatile uint8_t rear_pwm_duty = 0; // PH6

// Initialize pins and Timer3 for software PWM
void lysInit(void)
{
    // Configure output pins
    DDRB |= (1 << PB4); // headlight pin (PB4)
    DDRH |= (1 << PH6); // rear/brake pin (PH6)

    // Timer3: CTC mode to generate ISR for software PWM timing
    // Use prescaler = 8 and OCR3A = 14 -> ISR ~143 kHz
    // PWM frequency ~= ISR_freq / 256 ~= 560 Hz
    TCCR3A = 0; // normal port operation, CTC in TCCR3B
    TCCR3B = (1 << WGM32) | (1 << CS31); // WGM32=1 (CTC), prescaler=8
    OCR3A = 14;
    TIMSK3 |= (1 << OCIE3A); // enable compare A interrupt for Timer3

    // Ensure outputs start low
    PORTB &= ~(1 << PB4);
    PORTH &= ~(1 << PH6);

    // Enable global interrupts
    sei();
}

void turn_on_headlight(void)
{
    head_pwm_duty = 230; // 0..255
}

void turn_off_headlight(void)
{
    head_pwm_duty = 0;
    PORTB &= ~(1 << PB4);
}

void turn_on_rearlight(void)
{
    rear_pwm_duty=43.35;
}

void turn_off_rearlight(void)
{
    rear_pwm_duty = 0;
    PORTH &= ~(1 << PH6);
}

void turn_on_brakelight(void)
{
    rear_pwm_duty = 255;
}

// Timer3 Compare A interrupt - drive software PWM on PB4 and PH6
ISR(TIMER3_COMPA_vect)
{
    pwm_counter++; // 0..255 wrap

    // Headlight on PB4
    if (pwm_counter < head_pwm_duty)
        PORTB |= (1 << PB4);
    else
        PORTB &= ~(1 << PB4);

    // Rear / brake on PH6
    if (pwm_counter < rear_pwm_duty)
        PORTH |= (1 << PH6);
    else
        PORTH &= ~(1 << PH6);
}
