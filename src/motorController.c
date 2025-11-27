#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include "motorController.h"
#include "i2c.h"
#define PWM_PIN PB5  // Pin 11 → OCR1A
#define DIR_PIN PB6  // Pin 12 → Direction
#define PWM_RAMP_STEP 10  // PWM change step per tick


static volatile int g_current_pwm = 0;
static volatile int g_target_pwm = 0;
static volatile MotorDirection g_motor_retning = MOTOR_DIRECTION_DRIVE;

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


// Timer5: 100 Hz → soft PWM-change
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

    // Initialize Timer5 (100 Hz interrupt) to adjust PWM smoothly
    TCCR5A = 0;
    TCCR5B = 0;
    TCNT5 = 0;
    TIMSK5 = 0;

    OCR5A = 2499;                               // 16MHz / (64 * 100) - 1 = 100 Hz
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
// get current PWM value
int motorGetCurrentPWM(void) {
    return g_current_pwm;
}

// Timer1: Set PWM + direction
static void writePwm(uint16_t pwm, MotorDirection retning) {
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
    // If current PWM is less than target, increase it by PWM_RAMP_STEP
    if (current < target) {
        current += PWM_RAMP_STEP;
        if (current > target) {
            current = target;
        }
        // If current PWM is greater than target, decrease it by PWM_RAMP_STEP
    } else if (current > target) {
        current -= PWM_RAMP_STEP;
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
}
// Called from main to timeout the motor if needed
static void motorBreak(void) {
        g_target_pwm = 0;
    
}
// Reads temperature from LM75 sensor via I2C
static int GetCurrentTemp(void) {
    uint8_t temp_msb, temp_lsb;
    i2c_start();
    i2c_write(0x90); // LM75 address + write
    i2c_write(0x00); // Temperature register
    i2c_start();
    i2c_write(0x91); // LM75 address + read
    temp_msb = i2c_read(0); // Read MSB, send ACK
    temp_lsb = i2c_read(1); // Read LSB, send NACK
    i2c_stop();

    int16_t temp_raw = ((int16_t)temp_msb << 8) | temp_lsb;
    // Right shift by 7 to get temperature in 0.5 degree C units
    return temp_raw >> 7;

}
