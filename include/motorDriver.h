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

// Sæt mål: retning + PWM (0–1023)
void motorSetTarget(MotorDirection retning, int targetPWM);

// Set ramp speed
void motorSetRampSpeed(uint16_t step);

// Getters

// Hent nuværende PWM-værdi (den øjeblikkelige værdi under ramping)
int motorGetCurrentPWM(void);

// Hent mål PWM-værdi
int motorGetTargetPWM(void);

// Get current service ticks (counter)
uint16_t motorGetTicks(void);


// Timeout.
void motorBreak(void);

// wrapper using default ramp time.
void motorChangeDirection(MotorDirection new_dir, int targetPWM);

// --- Hardware Control ---

// void motorEnableOutput(void);
// void motorDisableOutput(void);

#endif
