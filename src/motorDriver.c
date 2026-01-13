#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <stdint.h>
#include <util/delay.h>
#include "motorController.h"
#include "lysdriver.h"

// --- Configuration ---
#define PWM_PIN         PB5
#define DIR_PIN         PB6
#define PWM_RAMP_DEFAULT 20
#define DIR_RAMP_MS_DEF  500

// Timer 5 Settings (200 Hz)
// 16MHz / (64 * 200Hz) - 1 = 1249
#define T5_PRESCALER    1249 
#define TIMEOUT_TICKS   12000 // 60 seconds at 200Hz

// --- State Types ---
typedef enum {
    DIR_STATE_IDLE = 0,
    DIR_STATE_RAMP_DOWN,
    DIR_STATE_WAIT
} DirStage;

typedef struct {
    bool active;
    DirStage stage;
    uint16_t startTick;
    uint16_t durationTicks;
    MotorDirection nextDir;
    int nextTargetPWM;
} DirChangeState;

// --- Global Variables ---
static volatile int g_current_pwm = 0;
static volatile int g_target_pwm = 0;
static volatile uint16_t g_ramp_step = PWM_RAMP_DEFAULT;
static volatile MotorDirection g_motor_retning = MOTOR_DIRECTION_DRIVE;
static volatile uint16_t g_service_ticks = 0;
static volatile DirChangeState g_dirState = {0}; // Initialize to 0/False

// Saved hardware state
static volatile bool g_pwm_enabled = true;
static uint8_t g_saved_com1a = 0;

// --- Helper Functions ---

static inline uint16_t limitPWM(int pwm) {
    if (pwm < 0) return 0;
    if (pwm > 1023) return 1023;
    return (uint16_t)pwm;
}

static void writePwm(uint16_t pwm, MotorDirection retning) {
    OCR1A = pwm; // Set PWM Duty Cycle
    
    if (retning == MOTOR_DIRECTION_DRIVE) {
        PORTB &= ~(1 << DIR_PIN);
    } else {
        PORTB |= (1 << DIR_PIN);
    }
}

// --- Interrupt Service Routine ---
ISR(TIMER5_COMPA_vect) {
    motorServiceTick();
}

// --- Core Logic ---

static inline void motorServiceTick(void) {
    // Handle Ramping
    int current = g_current_pwm;
    int target = g_target_pwm;
    //  Current * 2 + g_ramp_step
    change = (current * 2) + g_ramp_step;
    if (current != target) {
       
        if (current < target) {
            current += change;
            if (current > target) current = target;
        } else {
            // Decelerate
            current -= change
            if (current < target) current = target;
        }
        
        g_current_pwm = limitPWM(current);
        writePwm(g_current_pwm, g_motor_retning);
    }

    g_service_ticks++;

    // Handle Safe Direction Change State Machine
    if (g_dirState.active) {
        uint16_t elapsed = g_service_ticks - g_dirState.startTick;

        switch (g_dirState.stage) {
            case DIR_STATE_RAMP_DOWN:
                // Wait until PWM hits 0 OR timeout
                if (g_current_pwm == 0 || elapsed >= g_dirState.durationTicks) {
                    g_dirState.stage = DIR_STATE_WAIT;
                    g_dirState.startTick = g_service_ticks; // Reset timer for wait phase
                }
                break;

            case DIR_STATE_WAIT:
                // Wait approx 50ms (10 ticks) before reversing
                if (elapsed >= 10) {
                    motorSetRampSpeed(PWM_RAMP_DEFAULT);
                    motorSetTarget(g_dirState.nextDir, g_dirState.nextTargetPWM);
                    
                    // Reset State
                    g_dirState.active = false;
                    g_dirState.stage = DIR_STATE_IDLE;
                }
                break;
                
            default: break;
        }
    }

    // Timeout (60 Seconds)
    if (g_service_ticks >= TIMEOUT_TICKS) {
        g_target_pwm = 0;
        g_current_pwm = 0;
        motorBreak()
    }
}

// --- Public Functions ---
void resetTicks(void) {
    g_service_ticks = 0
}
void motorInit(void) {
    // Pin Config: PWM and DIR as Outputs
    DDRB |= (1 << PWM_PIN) | (1 << DIR_PIN);
    PORTB &= ~(1 << DIR_PIN);

    // Timer1: Fast PWM, 10-bit, Prescaler 8
    // WGM13:0 = 7 (Fast PWM 10-bit), CS12:0 = 2 (Prescaler 8)
    TCCR1A = (1 << COM1A1) | (1 << WGM11) | (1 << WGM10); // Clear on Compare
    TCCR1B = (1 << WGM12) | (1 << CS11);
    
    OCR1A = 0;
    g_saved_com1a = TCCR1A & ((1 << COM1A1) | (1 << COM1A0)); // Save enable bits

    // Timer5: CTC Mode, 200Hz Loop
    TCCR5A = 0;
    TCCR5B = (1 << WGM52) | (1 << CS51) | (1 << CS50); // CTC, Prescaler 64
    OCR5A = T5_PRESCALER;
    TIMSK5 = (1 << OCIE5A); // Enable Interrupt

    sei();
}

void motorSetTarget(MotorDirection retning, int targetPWM) {
    if(retning != g_motor_retning){
        motorChangeDirection(retning, targetPWM)
    }
    else{    
        g_target_pwm = limitPWM(targetPWM);
    }

}

void motorSetRampSpeed(uint16_t step) {
    g_ramp_step = step;
}

int motorGetCurrentPWM(void) {
    return g_current_pwm;
}

int motorGetTargetPWM(void) {
    return g_target_pwm;
}

void motorChangeDirectionSafely(MotorDirection new_dir, int targetPWM, uint16_t ramp_ms) {
    uint8_t sreg = SREG;
    cli();
    
    // Setup state machine
    g_dirState.active = true;
    g_dirState.stage = DIR_STATE_RAMP_DOWN;
    g_dirState.nextDir = new_dir;
    g_dirState.nextTargetPWM = targetPWM;
    g_dirState.durationTicks = (ramp_ms + 4) / 5; // Convert ms to ticks
    g_dirState.startTick = g_service_ticks;
    
    SREG = sreg;

    // Trigger ramp down
    motorSetRampSpeed(15);
    motorSetTarget(g_motor_retning, 0);
}

void motorChangeDirection(MotorDirection new_dir, int targetPWM) {
    motorChangeDirectionSafely(new_dir, targetPWM, DIR_RAMP_MS_DEF);
}

void motorBreak(void) {
    g_target_pwm = 0;
}
// til debugging / modultest
void motorEnableOutput(void) {
    uint8_t sreg = SREG;
    cli();
    // Restore bits to re-connect Timer to Pin
    TCCR1A = (TCCR1A & ~0xC0) | g_saved_com1a;
    DDRB |= (1 << PWM_PIN) | (1 << DIR_PIN);
    PORTB &= ~(1 << PWM_PIN); // Clear manual setting
    g_pwm_enabled = true;
    SREG = sreg;
}
// til debugging / modultest
void motorDisableOutput(void) {
    uint8_t sreg = SREG;
    cli();
    // Disconnect Timer from Pin
    TCCR1A &= ~((1 << COM1A1) | (1 << COM1A0)); 
    OCR1A = 0;
    
    // Set pins to safe state (Input or Low)
    PORTB &= ~((1 << PWM_PIN) | (1 << DIR_PIN));
    DDRB &= ~(1 << PWM_PIN); // Float PWM pin
    DDRB |= (1 << DIR_PIN);  // Drive Dir Low
    
    g_pwm_enabled = false;
    SREG = sreg;
}

uint16_t motorGetTicks(void) {
    uint16_t ticks;
    uint8_t sreg = SREG;
    cli();
    ticks = g_service_ticks;
    SREG = sreg;
    return ticks;
}
