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
// Number of service ticks to keep brake light on after a braking event
#define BRAKE_HOLD_TICKS 100
#define BRAKE_END_DELAY 100
#define BRAKE_OFF true
void koerBanen(void);

int main(void) {
    initSwitchPort();
    initLEDport();
    uartInit();
    baglys_init();    
    sensorInit();
    somo_init();
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
    uint16_t last_light_toggle = 0;
    uint16_t brake_latch_until = 0;

    while (running) {
        uint16_t current_ticks = motorGetTicks();

        sensorUpdate();
        uint16_t count = getReflexCount();

        if (count != last_count) {

            last_count = count;
            play_refleks_sound();

            if (count == 1) {
                motorSetRampSpeed(30);
                motorSetTarget(MOTOR_DIRECTION_DRIVE, UP_SPEED);
            }
            else if (count == 2) {
                motorSetRampSpeed(30);
                motorSetTarget(MOTOR_DIRECTION_DRIVE, UP_SPEED);
            }
            else if (count == 3) 
            {
                motorSetRampSpeed(25);
                motorSetTarget(MOTOR_DIRECTION_DRIVE, DOWN_SPEED);
            }
            else if (count == 4)
            {
                motorSetRampSpeed(30);
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
                
            }
        }
        
        // iflg. chat skal det være her, den slutter
        // Lysstyring

        int current_pwm = motorGetCurrentPWM();
        int target_pwm = motorGetTargetPWM();
        bool currently_braking = (current_pwm > target_pwm);
        
        // Set brake latch timer when transitioning from braking to not braking OR when PWM reaches 0
        static bool was_braking = false;
        static bool was_moving = false;
        if (was_braking && !currently_braking) {
            brake_latch_until = (uint16_t)(motorGetTicks() + BRAKE_HOLD_TICKS);
        }
        if (was_moving && current_pwm == 0) {
            brake_latch_until = (uint16_t)(motorGetTicks() + BRAKE_HOLD_TICKS);
        }
        was_braking = currently_braking;
        was_moving = (current_pwm > 0);

        // Update lights immediately if count < 1, otherwise throttle to every 10 ticks
        if (count < 1 || (uint16_t)(motorGetTicks() - last_light_toggle) >= 10) {
            last_light_toggle = motorGetTicks();

            // Brake should be on if currently braking or latch hasn't expired
            bool brake_latch_active = ((int16_t)(brake_latch_until - motorGetTicks()) > 0);
            bool brake_on = currently_braking || brake_latch_active;
            
            if (brake_on) {
                turn_on_brakelight();
                turn_on_headlight();
            } else if (current_pwm > 0 ) {
                turn_on_rearlight();
                turn_on_headlight();
            } else {
            }
            // End running as the last thing
            if (current_pwm == 0 && count >= 11 && !brake_on) {
                turn_off_rearlight();
                turn_off_headlight();
                running = false;
            }
        }

        
    }
}