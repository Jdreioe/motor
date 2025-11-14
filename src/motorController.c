#include <avr/io.h>
#include <avr/interrupt.h>
#include "motorController.h"

#define PWM_PIN PB5  // Pin 11 → OCR1A
#define DIR_PIN PB6  // Pin 12 → Direction

static volatile int g_current_pwm = 0;
static volatile int g_target_pwm = 0;
static volatile MotorRetning g_motor_retning = MOTOR_RETNING_FREM;

// Timer5: 1 kHz → soft PWM-change
ISR(TIMER5_COMPA_vect) {
    if (g_current_pwm < g_target_pwm) {
        g_current_pwm++;
    } else if (g_current_pwm > g_target_pwm) {
        g_current_pwm--;
    }
}
// Initialize Timer5 (1 kHz interrupt) to adjust PWM smoothly
void motorTimer5Init(void) {
    sei();
    TCCR5B |= (1 << WGM52);                     // CTC mode
    TCCR5B |= (1 << CS51) | (1 << CS50);        // Prescaler 64
    OCR5A = 249;                                // 16MHz / (64 * 1000) - 1 = 1 kHz
    TIMSK5 |= (1 << OCIE5A);                    // Enable interrupt
}
// Set target: direction + PWM (0–1023)
void motorSetTarget(MotorRetning retning, int targetPWM) {
    g_motor_retning = retning;
    if (targetPWM < 0) targetPWM = 0;
    if (targetPWM > 1023) targetPWM = 1023;
    g_target_pwm = targetPWM;
}
// get current PWM value
int motorGetCurrentPWM(void) {
    return g_current_pwm;
}

// initialize Timer1 (10-bit Fast PWM, ~1.95 kHz) to control motor
static void timer1_pwm_init(void) {
    DDRB |= (1 << PWM_PIN) | (1 << DIR_PIN);
    // Set Timer1 for 10-bit Fast PWM, prescaler 8
    TCCR1A = 0b00000011; // WGM11 | WGM10
    TCCR1B = 0b00001010; // WGM12 | CS11
    OCR1A = 0;
}
// SET PWM + direction
void motor_apply_output(MotorRetning retning, int pwm) {
    // If pwm is 0, disable PWM output
    

    // Ensure Timer1 is initialized
    if (!(TCCR1B & (1 << CS11))) {
        timer1_pwm_init();
    }
    // Ensure OC1A is connected
    if (!(TCCR1A & (1 << COM1A1))) {
        TCCR1A |= (1 << COM1A1);   // Connect OC1A
    }
    // Clamp and set PWM value on OCR1A (10-bit)
    if (pwm < 0) {
        pwm = 0;
    } else if (pwm > 1023) {
        pwm = 1023;
    }
    OCR1A = (uint16_t)pwm;
    // Set direction pin
    if (retning == MOTOR_RETNING_FREM) {
        PORTB &= ~(1 << DIR_PIN);
    } else {
        PORTB |= (1 << DIR_PIN);
    }
}

