#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include "motorController.h"
#include "switch.h"
#include "led.h"

#define TEST_SWITCH 0  // SW0

typedef struct {
    MotorRetning direction;
    int target_pwm;
    uint16_t duration_ms;
} MotorTestStep;

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

static void delay_ms(uint16_t ms) {
    while (ms--) _delay_ms(1);
}

static void wait_for_switch_press(void) {
    while (!switchOn(TEST_SWITCH)) _delay_ms(10);
    _delay_ms(40);
    while (switchOn(TEST_SWITCH)) _delay_ms(10);
    _delay_ms(40);
}

static void run_step(const MotorTestStep *step, unsigned char step_index) {
    unsigned char led_idx = step_index % 4;
    writeAllLEDs(1 << led_idx);
    wait_for_switch_press();
    writeAllLEDs((1 << led_idx) | (1 << 3));

    motorSetTarget(step->direction, step->target_pwm);

    uint32_t elapsed = 0;
    while (elapsed < step->duration_ms) {
        motor_apply_output(step->direction, motorGetCurrentPWM());
        _delay_ms(10);
        elapsed += 10;
    }

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

    motorSetTarget(MOTOR_RETNING_FREM, 0);

    writeAllLEDs(0x01); _delay_ms(300);
    writeAllLEDs(0x00); _delay_ms(300);

    while (1) {
        for (unsigned char i = 0; i < sizeof(kTestSteps)/sizeof(kTestSteps[0]); i++) {
            run_step(&kTestSteps[i], i);
        }
        writeAllLEDs(0x0F); _delay_ms(400);
        writeAllLEDs(0x00); _delay_ms(400);
    }
    return 0;
}