#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <stdint.h>
#include "motorController.h"
#include <util/delay.h>
#include "lysdriver.h"
#define DIRECTION_RAMP_MS_DEFAULT 500
#define PWM_PIN PB5  // Pin 11 → OCR1A
#define DIR_PIN PB6  // Pin 12 → Direction
#define PWM_RAMP_STEP_DEFAULT 20 // Default PWM change step per tick

// Forward declarations
static void writePwm(uint16_t pwm, MotorDirection retning);
static inline void motorServiceTick(void);

static volatile int g_current_pwm = 0;
static volatile int g_target_pwm = 0;
static volatile uint16_t g_ramp_step = PWM_RAMP_STEP_DEFAULT;
static volatile MotorDirection g_motor_retning = MOTOR_DIRECTION_DRIVE;
// Counts service ticks (incremented in motorServiceTick). Each tick = 10 ms
static volatile uint16_t g_service_ticks = 0;
// PWM output enabled state and saved COM1A bits
static volatile bool g_pwm_output_enabled = true;
static uint8_t g_saved_com1a_bits = 0;

// Limits PWM to 0-1023 range. If above, its set to 1023. If below, its set to 0. Otherwise, the value is returned unchanged.
static inline uint16_t limitPWM(int pwm) {
    if (pwm < 0) {
        return 0;
    }
    if (pwm > 1023) {
        return 1023;
    }
    return (uint16_t)pwm;
}


// Timer5: 200 Hz → soft PWM-change
ISR(TIMER5_COMPA_vect) {
    motorServiceTick();

}

// Initializes Motor Controller (Timer1 for PWM + Timer5 for Control Loop)
void motorInit(void) {
    // Configures pins
    DDRB |= (1 << PWM_PIN) | (1 << DIR_PIN);
    PORTB &= ~(1 << DIR_PIN);

    // Configures Timer1: Fast PWM 10-bit, Prescaler 8
    // WGM13:0 = 0111 (Fast PWM, 10-bit) -> WGM12=1, WGM11=1, WGM10=1
    // CS12:0 = 010 (Prescaler 8)
    // COM1A1:0 = 10 (Clear OC1A on Compare Match)
    TCCR1A = 0b10000011;
    TCCR1B = 0b00001010;
    
    OCR1A = 0;
    // Save COM1A bits so we can disconnect/reconnect OC1A later
    g_saved_com1a_bits = TCCR1A & 0xC0; // COM1A1:0 are bits 7..6

    // Initialize Timer5 (200 Hz interrupt) to adjust PWM smoothly
    TCCR5A = 0;
    TCCR5B = 0;
    TCNT5 = 0;
    TIMSK5 = 0;

    OCR5A = 1249;                               // 16MHz / (64 * 100) - 1 = 200 Hz
    TCCR5B = 0b00001011;                        // CTC mode, prescaler 64
    TIMSK5 = 0b00000010;                        // Enable interrupt

    sei();
}
// Set target: direction + PWM (0–1023)
void motorSetTarget(MotorDirection retning, int targetPWM) {
    uint16_t limitedPwm = limitPWM(targetPWM);
    g_motor_retning = retning;
    g_target_pwm = limitedPwm;
}

// Set ramp speed (step size per tick).
void motorSetRampSpeed(uint16_t step) {
    g_ramp_step = step;
}

// get current PWM value
int motorGetCurrentPWM(void) {
    return g_current_pwm;
}

// get target PWM value
int motorGetTargetPWM(void) {
    return g_target_pwm;
}

// Timer1: Set PWM + direction
static void writePwm(uint16_t pwm, MotorDirection retning) {
    // Always update OCR1A so timer compare value stays in sync. If output
    // is disabled the pin won't be driven because COM1A bits are cleared.
    OCR1A = pwm;

    if (retning == MOTOR_DIRECTION_DRIVE) {
        PORTB &= ~(1 << DIR_PIN);
    } else {
        PORTB |= (1 << DIR_PIN);
    }
}
// Loop: called from Timer5 ISR every 100 Hz
static inline void motorServiceTick(void) {
    int current = g_current_pwm;
    int target = g_target_pwm;
    // If current PWM is less than target, increase it by g_ramp_step
    if (current < target) {
        int ramp_calc = current * 2 + g_ramp_step;
        current += ramp_calc;
        if (current > target) {
            current = target;
        }
        // If current PWM is greater than target, decrease it by g_ramp_step
    } else if (current > target) {
        current -= g_ramp_step;
        if (current < target) {
            current = target;
        }
    }
    // Apply new PWM value, but limit it first
    uint16_t pwm = limitPWM(current);
    // Update global current PWM
    g_current_pwm = pwm;
    // Write to hardware
    writePwm(pwm, g_motor_retning);


    // Increment service tick counter (200 Hz -> 5 ms per tick)
    g_service_ticks++;

    // 1-minute timeout check (12000 ticks at 200 Hz= 60s)
    if (g_service_ticks >= 12000) {
        g_target_pwm = 0;
        g_current_pwm = 0;
        writePwm(0, g_motor_retning);
        
    }
}
// Called from main to timeout the motor if needed
void motorBreak(void) {
        g_target_pwm = 0;
    
}
// Safely change direction: ramp down to 0 (within `ramp_ms`), switch direction, ramp up to `targetPWM`.
static void motorChangeDirectionSafely(MotorDirection new_dir, int targetPWM, uint16_t ramp_ms) {
    // Compute max number of service ticks to wait (each tick = 5 ms)
    uint16_t start = g_service_ticks;
    uint16_t max_ticks = (ramp_ms + 4) / 5; // ceil(ramp_ms/5)

    // Request ramp down to 0 while keeping current direction
    motorSetRampSpeed(15);
    motorSetTarget(g_motor_retning, 0);

    // Wait until current PWM reaches 0 or timeout
    while (g_current_pwm > 0) {
        if ((uint16_t)(g_service_ticks - start) >= max_ticks) {
            break; // timeout
        }
        // busy-wait; service ticks are incremented in ISR
    }

    // Wait for 1000ms (1000 ticks) before starting againb
    uint16_t delay_start = g_service_ticks;
    while ((uint16_t)(g_service_ticks - delay_start) < 200) {
        // busy-wait
    }

    // Now set new direction and target PWM (will ramp up by service ISR)
    motorSetRampSpeed(5);
    motorSetTarget(new_dir, targetPWM);
}

// Convenience wrapper using default macro from header
void motorChangeDirection(MotorDirection new_dir, int targetPWM) {
    motorChangeDirectionSafely(new_dir, targetPWM, DIRECTION_RAMP_MS_DEFAULT);
}

// Enable the OC1A pin output by restoring COM1A bits
void motorEnableOutput(void) {
    // Restore saved COM1A bits atomically
    uint8_t sreg = SREG;
    cli();
    TCCR1A = (TCCR1A & ~0xC0) | (g_saved_com1a_bits & 0xC0);
    // Ensure PWM and DIR pins are outputs again
    DDRB |= (1 << PWM_PIN) | (1 << DIR_PIN);
    // Clear PWM pin output value (OCR1A controls actual waveform when enabled)
    PORTB &= ~(1 << PWM_PIN);
    g_pwm_output_enabled = true;
    SREG = sreg;
}

// Disable the OC1A pin output by clearing COM1A bits. Also set OCR1A=0
// to ensure no residual compare value.
void motorDisableOutput(void) {
    uint8_t sreg = SREG;
    cli();
    // Disconnect OC1A from the pin
    TCCR1A &= ~0xC0; // Clear COM1A1:0
    OCR1A = 0;
    // Ensure the pin does not drive the motor: clear output value and
    // make the PWM pin an input (tri-state) to avoid any residual drive
    PORTB &= ~(1 << PWM_PIN);
    DDRB &= ~(1 << PWM_PIN);
    // Also set direction pin to a defined safe state (drive low)
    PORTB &= ~(1 << DIR_PIN);
    DDRB |= (1 << DIR_PIN);
    g_pwm_output_enabled = false;
    SREG = sreg;
}

// Get current service ticks (200Hz counter)
uint16_t motorGetTicks(void) {
    uint16_t ticks;
    uint8_t sreg = SREG;
    cli();
    ticks = g_service_ticks;
    SREG = sreg;
    return ticks;
}
uint16_t motorGetTicks(void) {
    uint16_t ticks;
    uint8_t sreg = SREG;
    cli();
    ticks = g_service_ticks;
    SREG = sreg;
    return ticks;
}
