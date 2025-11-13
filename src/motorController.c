#include <avr/io.h>
#include <avr/interrupt.h>
#include "motorController.h"

#define PWM_PIN PB5  // Pin 11 → OCR1A
#define DIR_PIN PB6  // Pin 12 → Retning

static volatile int g_current_pwm = 0;
static volatile int g_target_pwm = 0;
static volatile MotorRetning g_motor_retning = MOTOR_RETNING_FREM;

// Timer5: 1 kHz → blød PWM-ændring
ISR(TIMER5_COMPA_vect) {
    if (g_current_pwm < g_target_pwm) {
        g_current_pwm++;
    } else if (g_current_pwm > g_target_pwm) {
        g_current_pwm--;
    }
}

void motorTimer5Init(void) {
    sei();
    TCCR5B |= (1 << WGM52);                     // CTC mode
    TCCR5B |= (1 << CS51) | (1 << CS50);        // Prescaler 64
    OCR5A = 249;                                // 16MHz / (64 * 1000) - 1 = 1 kHz
    TIMSK5 |= (1 << OCIE5A);                    // Enable interrupt
}

void motorSetTarget(MotorRetning retning, int targetPWM) {
    g_motor_retning = retning;
    if (targetPWM < 0) targetPWM = 0;
    if (targetPWM > 255) targetPWM = 255;
    g_target_pwm = targetPWM;
}

int motorGetCurrentPWM(void) {
    return g_current_pwm;
}

// Initialiser Timer1 (8-bit Fast PWM, 31.25 kHz)
static void timer1_pwm_init(void) {
    DDRB |= (1 << PWM_PIN) | (1 << DIR_PIN);
    TCCR1A = (1 << WGM10);                      // 8-bit Fast PWM
    TCCR1B = (1 << WGM12) | (1 << CS11);        // Prescaler 8 → ~31.25 kHz
    OCR1A = 0;
}

void motor_apply_output(MotorRetning retning, int pwm) {
    if (pwm == 0) {
        TCCR1A &= ~(1 << COM1A1);  // Frakobl OC1A
        PORTB &= ~(1 << PWM_PIN);
        PORTB &= ~(1 << DIR_PIN);
        OCR1A = 0;
        return;
    }

    if (!(TCCR1B & (1 << CS11))) {
        timer1_pwm_init();
    }

    if (!(TCCR1A & (1 << COM1A1))) {
        TCCR1A |= (1 << COM1A1);   // Tilslut OC1A
    }

    OCR1A = (uint8_t)pwm;

    if (retning == MOTOR_RETNING_FREM) {
        PORTB &= ~(1 << DIR_PIN);
    } else {
        PORTB |= (1 << DIR_PIN);
    }
}

void motorSimulateISR(void) {
    if (g_current_pwm < g_target_pwm) g_current_pwm++;
    else if (g_current_pwm > g_target_pwm) g_current_pwm--;
}