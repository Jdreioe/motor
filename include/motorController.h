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

void motor_apply_output(MotorDirection retning, int pwm);

static void timer1WritePwm(uint16_t pwm, MotorDirection retning);

static inline void motorServiceTick(void);

static uint16_t limit_pwm(int pwm);

int GetCurrentTemp(void);

void motorPause(void);


#endif