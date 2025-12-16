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
#include "somo.h"
#include "lysdriver.h"
#define PWM_PERCENT(pct) ((uint16_t)((pct) * 1023UL / 100UL))
#define NORM_SPEED PWM_PERCENT(70)
#define UP_SPEED PWM_PERCENT(100)
#define DOWN_SPEED PWM_PERCENT(40)
#define STOP 0


void koerBanen(void);

int main(void) {
    initSwitchPort();
    initLEDport();
    uartInit();
    baglys_init();    
    sensorInit();
    somoInit();
    motorInit();

    printf("System Initialized\n");

    while (1) {
        // Wait for start signal (e.g., SW0)
        while (!switchOn(0)) {
            _delay_ms(10);
        }

         koerBanen();
        
        // Debounce/wait before next run
        _delay_ms(1000);
    }
    
    return 0;
}

void koerBanen(void) {
    resetReflexCount();
    motorSetTarget(MOTOR_DIRECTION_DRIVE, 0);
    // Start driving forward
    motorSetTarget(MOTOR_DIRECTION_DRIVE, NORM_SPEED);

    uint16_t last_count = 0xFFFF;
    bool running = true;
    bool braking = false;
    uint16_t brake_start_tick = 0;

    while (running) {
        uint16_t current_ticks = motorGetTicks();

        sensorUpdate();
        uint16_t count = getReflexCount();

        if (count != last_count) {

            last_count = count;
            play_refleks_sound();

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
                play_finish_sound();
                _delay_ms(200);
 
                motorSetTarget(MOTOR_DIRECTION_DRIVE, STOP);
                 
                running = false;                
            }
        } // iflg. chat skal det være her, den slutter
        
        // Lysstyring
        int current_pwm = motorGetCurrentPWM();
        int target_pwm = motorGetTargetPWM();

        if (current_pwm > target_pwm) {
            turn_on_brakelight();
            turn_on_headlight();
           uint16_t delay_start = motorGetTicks();
    while ((uint16_t)(motorGetTicks() - delay_start) < 100) {
        // busy-wait
    }
        // Hvis der er spænding på motoren
        } else if (current_pwm > 0) {
            turn_on_rearlight();
            turn_on_headlight();
            // Ellers sluk lys
        } else {
            turn_off_rearlight();
            turn_off_headlight();
        }

    }
}