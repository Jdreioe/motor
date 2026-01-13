#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H

typedef enum {
    MOTOR_DIRECTION_DRIVE,
    MOTOR_DIRECTION_REVERSE
} MotorDirection; 

// Initialiser Motor (Timer1 PWM + Control Loop)
void motorInit(void);

// Core Control

// Reset serviceTicks to 0
void resetTicks(void);

// Set target PWM and direction
void motorSetTarget(MotorDirection direction, int targetPWM);

// Set ramp speed
void motorSetRampSpeed(uint16_t step);

// Getters

// Get current PWM
int motorGetCurrentPWM(void);

// get PWM target
int motorGetTargetPWM(void);

// Get current service ticks (counter)
uint16_t motorGetTicks(void);


// Timeout.
void motorBreak(void);

// Change direction.
void motorChangeDirection(MotorDirection new_dir, int targetPWM);

// --- Hardware Control ---

// void motorEnableOutput(void);
// void motorDisableOutput(void);

#endif
