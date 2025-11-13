#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include "motorController.h"
#include "switch.h"
#include "led.h"

#define TEST_SWITCH 0  // SW0
// Struct for each test step
typedef struct {
    MotorRetning direction;
    int target_pwm;
    uint16_t duration_ms;
} MotorTestStep;
// Testsequence
static const MotorTestStep kTestSteps[] = {
    { MOTOR_RETNING_FREM,  26, 3000 },
    { MOTOR_RETNING_FREM,   0, 3000 },
    { MOTOR_RETNING_BAK,   26, 3000 },
    { MOTOR_RETNING_FREM,  51, 3000 },
    { MOTOR_RETNING_BAK,   51, 3000 },
    { MOTOR_RETNING_FREM,  77, 3000 },
    { MOTOR_RETNING_BAK,   77, 3000 },
    { MOTOR_RETNING_FREM, 102, 3000 },
    { MOTOR_RETNING_BAK,  102, 3000 },
    { MOTOR_RETNING_FREM, 128, 3000 },
    { MOTOR_RETNING_BAK,  128, 3000 },
    { MOTOR_RETNING_FREM, 153, 3000 },
    { MOTOR_RETNING_BAK,  153, 3000 },
    { MOTOR_RETNING_FREM, 179, 3000 },
    { MOTOR_RETNING_BAK,  179, 3000 },
    { MOTOR_RETNING_FREM, 204, 3000 },
    { MOTOR_RETNING_BAK,  204, 3000 },
    { MOTOR_RETNING_FREM, 230, 3000 },
    { MOTOR_RETNING_BAK,  230, 3000 },
    { MOTOR_RETNING_FREM, 255, 3000 }
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
    initLEDport();
    motorTimer5Init();
    // Ensure motor is stopped at start
    motorSetTarget(MOTOR_RETNING_FREM, 0);

    writeAllLEDs(0x01); _delay_ms(300);
    writeAllLEDs(0x00); _delay_ms(300);
    // Main test loop
    while (1) {
        for (unsigned char i = 0; i < sizeof(kTestSteps)/sizeof(kTestSteps[0]); i++) {
            run_step(&kTestSteps[i], i);
        }
        writeAllLEDs(0x0F); _delay_ms(400);
        writeAllLEDs(0x00); _delay_ms(400);
    }
    return 0;
}