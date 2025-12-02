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

// Hent nuværende PWM-værdi
int motorGetCurrentPWM(void);

// Default ramp time (ms) used for safe direction changes.
#define DIRECTION_RAMP_MS_DEFAULT 500

// Change direction safely: ramp down, switch direction, ramp up.
// `ramp_ms` is the maximum time allowed for ramping down (in ms).
void motorChangeDirectionSafely(MotorDirection new_dir, int targetPWM, uint16_t ramp_ms);

// Convenience wrapper using default ramp time.
void motorChangeDirection(MotorDirection new_dir, int targetPWM);

// Enable/disable the PWM output pin. When disabled the OC1A output
// is disconnected from the pin (COM1A bits cleared) to ensure the
// pin is not driving the motor when no function is selected.
void motorEnableOutput(void);
void motorDisableOutput(void);

static void WritePwm(uint16_t pwm, MotorDirection retning);

static inline void motorServiceTick(void);

static uint16_t limit_pwm(int pwm);

static int GetCurrentTemp(void);

void motorPause(void);


#endif