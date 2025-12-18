#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

typedef enum {
    MOTOR_DIRECTION_DRIVE,
    MOTOR_DIRECTION_REVERSE
} MotorDirection;

// Initialiser Motor (Timer1 PWM + Control Loop)
void motorInit(void);

// Sæt mål: retning + PWM (0–1023)
void motorSetTarget(MotorDirection retning, int targetPWM);

// Set ramp speed (step size per tick). Default is 20.
void motorSetRampSpeed(uint16_t step);

// Hent nuværende PWM-værdi
int motorGetCurrentPWM(void);

// Hent mål PWM-værdi
int motorGetTargetPWM(void);

// Default ramp time (ms) used for safe direction changes.
// Slukker motoren ved timeout
void motorBreak();

// Change direction safely: ramp down, switch direction, ramp up.
// `ramp_ms` is the maximum time allowed for ramping down (in ms).
static void motorChangeDirectionSafely(MotorDirection new_dir, int targetPWM, uint16_t ramp_ms);

// Convenience wrapper using default ramp time.
void motorChangeDirection(MotorDirection new_dir, int targetPWM);

// Enable/disable the PWM output pin. When disabled the OC1A output
// is disconnected from the pin (COM1A bits cleared) to ensure the
// pin is not driving the motor when no function is selected. (for debugginh)
void motorEnableOutput(void);
void motorDisableOutput(void);

// Get current service ticks (200Hz counter)
#include <stdbool.h>

uint16_t motorGetTicks(void);

#endif