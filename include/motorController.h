#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

typedef enum {
    MOTOR_RETNING_FREM,
    MOTOR_RETNING_BAK
} MotorRetning;

// Initialiser Timer5 (1 kHz interrupt)
void motorTimer5Init(void);

// Sæt mål: retning + PWM (0–1023)
void motorSetTarget(MotorRetning retning, int targetPWM);

// Hent nuværende PWM-værdi
int motorGetCurrentPWM(void);

// UDFØR: Sæt PWM + retning på hardware (sikker!)
void motor_apply_output(MotorRetning retning, int pwm);

// Til test: Simuler én ISR-iteration
void motorSimulateISR(void);

#endif