#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdio.h>
#include "motorController.h"
#include "switch.h"
#include "led.h"
#include "sensorDriver.h"
#include "uart.h"

#define PWM_PERCENT(pct) ((uint16_t)((pct) * 1023UL / 100UL))
#define NORM_SPEED PWM_PERCENT(60)
#define UP_SPEED PWM_PERCENT(100)
#define DOWN_SPEED PWM_PERCENT(40)
#define STOP 0


int main(void) {
    initSwitchPort();
    initLEDport();
    uartInit();
    motorInit();
    sensorInit();
    
    printf("System Initialized\n");

    // Ensure motor is stopped at start
    motorSetTarget(MOTOR_DIRECTION_DRIVE, 0);

    // Wait for start signal (e.g., SW0)
    while (!switchOn(0)) {
        _delay_ms(10);
    }
    printf("Starting...\n");
    _delay_ms(500); // Debounce/wait

    // Start driving forward
    motorSetTarget(MOTOR_DIRECTION_DRIVE, NORM_SPEED);

    uint16_t last_count = 0xFFFF;
    uint32_t loop_counter = 0;

    while (1) {

        sensorUpdate();
        uint16_t count = getReflexCount();

        if (count != last_count) {

            last_count = count;
        }

        if (count == 1) {
            motorSetTarget(MOTOR_DIRECTION_DRIVE, NORM_SPEED);
        }
        else if (count == 2) {
            motorSetTarget(MOTOR_DIRECTION_DRIVE, UP_SPEED);
        }
        else if (count == 3) 
        {
            motorSetTarget(MOTOR_DIRECTION_DRIVE, DOWN_SPEED);
        }
        else if (count == 4)
        {
            motorSetTarget(MOTOR_DIRECTION_DRIVE, NORM_SPEED);
        }
        
        else if (count == 6) {
            motorSetTarget(MOTOR_DIRECTION_REVERSE, NORM_SPEED);
        }
        else if (count == 7) {

            motorSetTarget(MOTOR_DIRECTION_DRIVE, STOP);
            _delay_ms(2000);
            motorSetTarget(MOTOR_DIRECTION_DRIVE, NORM_SPEED);
        }
        else if (count >= 8 && count < 10) {
            
            motorSetTarget(MOTOR_DIRECTION_DRIVE, NORM_SPEED);
        }
        else if (count >= 10) {

            motorSetTarget(MOTOR_DIRECTION_DRIVE, STOP);
        }
        
        _delay_ms(10);
    }
    
    return 0;
}

int GetCurrentTemp(void) {

}