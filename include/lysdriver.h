/* Lysdriver.h
 * Driver API for lights on ATmega2560
 * Mapping requested by user:
 *  - PB4 = headlight (software PWM)
 *  - PH6 = rear light + brake (software PWM on same pin)
 */
#ifndef LYSDRIVER_H
#define LYSDRIVER_H

#include <stdint.h>

// Initialize I/O and timers used by the driver.
void baglys_init(void);

// Headlight control (PB4)
void turn_on_headlight(uint8_t pwm);
void turn_off_headlight(void);

// Rear light / brake control (PH6)
void turn_on_rearlight(uint16_t pwm);
void turn_off_rearlight(void);
void turn_on_brakelight(uint16_t pwm);

#endif // LYSDRIVER_H
