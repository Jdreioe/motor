#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include "motorController.h"
#include "switch.h"
#include "led.h"

#define TEST_SWITCH 0  // SW0
#define PWM_PERCENT(pct) ((uint16_t)((pct) * 1023UL / 100UL))
// Struct for each test step
typedef struct {
    MotorRetning direction;
    uint16_t target_pwm;
    uint16_t duration_ms;
} MotorTestStep;
// Testsequence
static const MotorTestStep kTestSteps[] = {
    { MOTOR_RETNING_FREM, PWM_PERCENT(10), 3000 },
    { MOTOR_RETNING_BAK,  PWM_PERCENT(10), 3000 },
    { MOTOR_RETNING_FREM, PWM_PERCENT(25), 3000 },
    { MOTOR_RETNING_BAK,  PWM_PERCENT(25), 3000 },
    { MOTOR_RETNING_FREM, PWM_PERCENT(40), 3000 },
    { MOTOR_RETNING_BAK,  PWM_PERCENT(40), 3000 },
    { MOTOR_RETNING_FREM, PWM_PERCENT(55), 3000 },
    { MOTOR_RETNING_BAK,  PWM_PERCENT(55), 3000 },
    { MOTOR_RETNING_FREM, PWM_PERCENT(70), 3000 },
    { MOTOR_RETNING_BAK,  PWM_PERCENT(70), 3000 },
    { MOTOR_RETNING_FREM, PWM_PERCENT(85), 3000 },
    { MOTOR_RETNING_BAK,  PWM_PERCENT(85), 3000 },
    { MOTOR_RETNING_FREM, PWM_PERCENT(100), 3000 }
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
static void run_step(const MotorTestStep *step, unsigned char step_index) {
    // Calculate LED index based on step index
    unsigned char led_idx = step_index % 4;
    writeAllLEDs(1 << led_idx);
    wait_for_switch_press();
    writeAllLEDs((1 << led_idx) | (1 << 3));
    // Set motor target direction and PWM
    motorSetTarget(step->direction, step->target_pwm);
    // Run for specified duration
    uint32_t elapsed = 0;
    while (elapsed < step->duration_ms) {
        motor_apply_output(step->direction, motorGetCurrentPWM());
        _delay_ms(10);
        elapsed += 10;
    }
    // Stop motor before next step
    motorSetTarget(step->direction, 0);
    while (motorGetCurrentPWM() > 0) {
        motor_apply_output(step->direction, motorGetCurrentPWM());
        _delay_ms(10);
    }

    writeAllLEDs(0x00);
    _delay_ms(250);
}

int main(void) {
    initSwitchPort();
    //initLEDport();
    motorTimer5Init();
    // Ensure motor is stopped at start
    motorSetTarget(MOTOR_RETNING_FREM, 0);

    writeAllLEDs(0x01); _delay_ms(300);
    writeAllLEDs(0x00); _delay_ms(300);

    // When SW1 is pressed, set PWM to 50% constantly
    while (1) {
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