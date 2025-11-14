#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>
#include "motorController.h"
#include "switch.h"
#include "led.h"

#define TEST_SWITCH 0  // SW0
#define STOP_SWITCH 3  // SW3

#define PWM_PERCENT(pct) ((uint16_t)((pct) * 1023UL / 100UL))
// Struct for each test step
typedef struct {
    MotorRetning direction;
    uint16_t target_pwm;
    uint16_t duration_ms;
} MotorTestStep;
// Testsequence
static const MotorTestStep kTestSteps[] = {
    { MOTOR_RETNING_FREM, PWM_PERCENT(10), 5000 },
    { MOTOR_RETNING_BAK,  PWM_PERCENT(10), 5000 },
    { MOTOR_RETNING_FREM, PWM_PERCENT(25), 5000 },
    { MOTOR_RETNING_BAK,  PWM_PERCENT(25), 5000 },
    { MOTOR_RETNING_FREM, PWM_PERCENT(40), 5000 },
    { MOTOR_RETNING_BAK,  PWM_PERCENT(40), 5000 },
    { MOTOR_RETNING_FREM, PWM_PERCENT(55), 5000 },
    { MOTOR_RETNING_BAK,  PWM_PERCENT(55), 5000 },
    { MOTOR_RETNING_FREM, PWM_PERCENT(70), 5000 },
    { MOTOR_RETNING_BAK,  PWM_PERCENT(70), 5000 },
    { MOTOR_RETNING_FREM, PWM_PERCENT(85), 5000 },
    { MOTOR_RETNING_BAK,  PWM_PERCENT(85), 5000 },
    { MOTOR_RETNING_FREM, PWM_PERCENT(100), 5000 }
};
// Simple delay function
static void delay_ms(uint16_t ms) {
    while (ms--) _delay_ms(1);
}
// Wait for switch press and release
static void wait_for_switch_press(void) {
    while (!switchOn(TEST_SWITCH)) _delay_ms(10);
    _delay_ms(40);
    while (switchOn(TEST_SWITCH)) _delay_ms(10);
    _delay_ms(40);
}
// Run a single test step
static bool run_step(const MotorTestStep *step, unsigned char step_index) {
    // Calculate LED index based on step index
    unsigned char led_idx = step_index % 4;
    writeAllLEDs((1 << led_idx) | (1 << 3));
    // Set motor target direction and PWM
    motorSetTarget(step->direction, step->target_pwm);
    // Run for specified duration
    uint32_t elapsed = 0;
    bool stop_requested = false;
    while (elapsed < step->duration_ms) {
        if (switchOn(STOP_SWITCH)) {
            stop_requested = true;
            break;
        }
        motor_apply_output(step->direction, motorGetCurrentPWM());
        _delay_ms(10);
        elapsed += 10;
    }
    // Stop motor before next step
    motorSetTarget(step->direction, 0);
    while (motorGetCurrentPWM() > 0) {
        motor_apply_output(step->direction, motorGetCurrentPWM());
        if (switchOn(STOP_SWITCH)) {
            stop_requested = true;
            break;
        }
        _delay_ms(10);
    }

    motor_apply_output(step->direction, 0);

    writeAllLEDs(0x00);
    _delay_ms(250);
    return !stop_requested;
}

int main(void) {
    initSwitchPort();
    initLEDport();
    motorTimer5Init();
    // Ensure motor is stopped at start
    motorSetTarget(MOTOR_RETNING_FREM, 0);

    writeAllLEDs(0x01); _delay_ms(300);
    writeAllLEDs(0x00); _delay_ms(300);

    // SW0 triggers automated test sequence; SW1 provides manual 50% forward drive
    while (1) {
        if (switchOn(TEST_SWITCH)) {
            wait_for_switch_press();
            bool completed = true;
            for (unsigned char i = 0; i < (sizeof(kTestSteps) / sizeof(kTestSteps[0])); ++i) {
                if (!run_step(&kTestSteps[i], i)) {
                    completed = false;
                    break;
                }
            }
            motorSetTarget(MOTOR_RETNING_FREM, 0);
            motor_apply_output(MOTOR_RETNING_FREM, 0);
            if (!completed) {
                while (switchOn(STOP_SWITCH)) {
                    _delay_ms(10);
                }
            }
        }

        if (switchOn(1)) {
            motorSetTarget(MOTOR_RETNING_FREM, PWM_PERCENT(50));
            motor_apply_output(MOTOR_RETNING_FREM, motorGetCurrentPWM());
        } else {
            motorSetTarget(MOTOR_RETNING_FREM, 0);
            motor_apply_output(MOTOR_RETNING_FREM, 0);
        }
        _delay_ms(10);
    }
    return 0;
}