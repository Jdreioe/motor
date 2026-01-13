#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H

typedef enum {
    MOTOR_DIRECTION_DRIVE,
    MOTOR_DIRECTION_REVERSE
} MotorDirection;

// --- Initialization ---

// Initialiser Motor (Timer1 PWM + Control Loop)
void motorInit(void);

// --- Core Control ---

// Denne skal kaldes af din Timer5 ISR (f.eks. ved 200Hz eller 100Hz)
// Håndterer acceleration, deceleration og retningsskift.
void motorServiceTick(void);
// Reset serviceTicks
void resetTicks(void);

// Sæt mål: retning + PWM (0–1023)
void motorSetTarget(MotorDirection retning, int targetPWM);

// Set ramp speed (step size per tick). Default is 20.
// Bruges nu både til acceleration og deceleration.
void motorSetRampSpeed(uint16_t step);

// --- Getters ---

// Hent nuværende PWM-værdi (den øjeblikkelige værdi under ramping)
int motorGetCurrentPWM(void);

// Hent mål PWM-værdi
int motorGetTargetPWM(void);

// Get current service ticks (counter)
uint16_t motorGetTicks(void);

// --- Safety & Utilities ---

// Stopper motoren (Brake) / Nulstiller state.
// Bruges f.eks. ved timeout eller nødstop.
void motorBreak(void);

// Change direction safely: ramp down, switch direction, ramp up.
// `ramp_ms` is the maximum time allowed for ramping down (in ms).
void motorChangeDirectionSafely(MotorDirection new_dir, int targetPWM, uint16_t ramp_ms);

// Convenience wrapper using default ramp time.
void motorChangeDirection(MotorDirection new_dir, int targetPWM);

// --- Hardware Control ---

void motorEnableOutput(void);
void motorDisableOutput(void);

#endif
