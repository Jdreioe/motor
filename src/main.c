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
#define NORM_SPEED PWM_PERCENT(70)
#define UP_SPEED PWM_PERCENT(100)
#define DOWN_SPEED PWM_PERCENT(40)
#define STOP 0


int main(void) {
    initSwitchPort();
    initLEDport();
    uartInit();
    
    sensorInit();
    
    printf("System Initialized\n");

    // Ensure motor is stopped at start


    // Wait for start signal (e.g., SW0)
    while (!switchOn(0)) {
        _delay_ms(10);
    }
    printf("Starting...\n");
    motorInit();
    _delay_ms(500); // Debounce/wait

    motorSetTarget(MOTOR_DIRECTION_DRIVE, 0);
    // Start driving forward
    motorSetTarget(MOTOR_DIRECTION_DRIVE, NORM_SPEED);

    uint16_t last_count = 0xFFFF;
    uint32_t loop_counter = 0;

    while (1) {

        sensorUpdate();
        uint16_t count = getReflexCount();

        if (count != last_count) {

            last_count = count;
        // prtøver uden }
            if (count == 1) {
                motorSetRampSpeed(20);
                motorSetTarget(MOTOR_DIRECTION_DRIVE, UP_SPEED);
            }
            else if (count == 2) {
                motorSetRampSpeed(20);
                motorSetTarget(MOTOR_DIRECTION_DRIVE, UP_SPEED);
            }
            else if (count == 3) 
            {
                motorSetRampSpeed(25);
                motorSetTarget(MOTOR_DIRECTION_DRIVE, DOWN_SPEED);
            }
            else if (count == 4)
            {
                motorSetRampSpeed(20);
                motorSetTarget(MOTOR_DIRECTION_DRIVE, NORM_SPEED);
            }
            
            else if (count == 6) {  
                motorChangeDirection(MOTOR_DIRECTION_REVERSE, NORM_SPEED);
                }
            else if (count == 8) {
                motorChangeDirection(MOTOR_DIRECTION_DRIVE, NORM_SPEED);
            }

            else if (count == 11) {
                motorSetRampSpeed(20);
                _delay_ms(200);
                motorSetTarget(MOTOR_DIRECTION_DRIVE, STOP);
            }
        } // iflg. chat skal det være her, den slutter
        
        _delay_ms(10);
    }
    
    return 0;
}

int GetCurrentTemp(void) {

}