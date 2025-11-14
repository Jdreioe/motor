#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <stdbool.h>
#include "motorController.h"

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define UART_BAUD 9600UL
#define UART_UBRR_VALUE ((F_CPU / (16UL * UART_BAUD)) - 1UL)

#define PWM_PIN PB5  // Pin 11 → OCR1A
#define DIR_PIN PB6  // Pin 12 → Direction

#define PWM_SLEW_STEP 10

static volatile int g_current_pwm = 0;
static volatile int g_target_pwm = 0;
static volatile MotorRetning g_motor_retning = MOTOR_RETNING_FREM;
static bool s_timer1_initialized = false;
static bool s_uart_initialized = false;
static int s_last_reported_pwm = -1;
static MotorRetning s_last_reported_dir = MOTOR_RETNING_FREM;

static inline uint16_t clamp_pwm_to_10bit(int pwm) {
    if (pwm < 0) {
        return 0;
    }
    if (pwm > 1023) {
        return 1023;
    }
    return (uint16_t)pwm;
}

static void timer1_write_pwm(uint16_t pwm, MotorRetning retning);
static inline void motor_service_tick(void);
static void uart_init(void);
static void uart_write_byte(uint8_t data);
static void uart_write_string(const char *text);
static void uart_write_uint16(uint16_t value);
static void uart_report_current(uint16_t pwm, MotorRetning retning);

// Timer5: 100 Hz → soft PWM-change
ISR(TIMER5_COMPA_vect) {
    motor_service_tick();
}
// Initialize Timer5 (100 Hz interrupt) to adjust PWM smoothly
void motorTimer5Init(void) {
    cli();
    TCCR5A = 0;
    TCCR5B = 0;
    TCNT5 = 0;
    TIMSK5 = 0;

    OCR5A = 2499;                               // 16MHz / (64 * 100) - 1 = 100 Hz
    TCCR5B = (1 << WGM52) | (1 << CS51) | (1 << CS50); // CTC mode, prescaler 64
    TIMSK5 = (1 << OCIE5A);                    // Enable interrupt
    sei();

    timer1_write_pwm(clamp_pwm_to_10bit(g_current_pwm), g_motor_retning);
}
// Set target: direction + PWM (0–1023)
void motorSetTarget(MotorRetning retning, int targetPWM) {
    uint16_t clamped = clamp_pwm_to_10bit(targetPWM);
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        g_motor_retning = retning;
        g_target_pwm = clamped;
    }
}
// get current PWM value
int motorGetCurrentPWM(void) {
    int value;
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        value = g_current_pwm;
    }
    return value;
}

// initialize Timer1 (10-bit Fast PWM, ~1.95 kHz) to control motor
static void timer1_pwm_init(void) {
    DDRB |= (1 << PWM_PIN) | (1 << DIR_PIN);
    PORTB &= ~(1 << DIR_PIN);

    TCCR1A = 0;
    TCCR1B = 0;
    TCCR1C = 0;
    TCNT1 = 0;

    // 10-bit Fast PWM (mode 7), non-inverting on OC1A, prescaler 8 (~1.95 kHz)
    TCCR1A = (1 << COM1A1) | (1 << WGM11) | (1 << WGM10);
    TCCR1B = (1 << WGM12) | (1 << CS11);
    OCR1A = 0;

    s_timer1_initialized = true;
}
// SET PWM + direction
void motor_apply_output(MotorRetning retning, int pwm) {
    uint16_t clamped = clamp_pwm_to_10bit(pwm);
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        g_motor_retning = retning;
        g_current_pwm = clamped;
    }
    timer1_write_pwm(clamped, retning);
    uart_report_current(clamped, retning);
}

static void uart_init(void) {
    if (s_uart_initialized) {
        return;
    }

    UBRR0H = (uint8_t)(UART_UBRR_VALUE >> 8);
    UBRR0L = (uint8_t)(UART_UBRR_VALUE & 0xFF);
    UCSR0A = 0;
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

    s_uart_initialized = true;
}

static void uart_write_byte(uint8_t data) {
    while (!(UCSR0A & (1 << UDRE0))) {
        // Busy-wait until TX buffer available
    }
    UDR0 = data;
}

static void uart_write_string(const char *text) {
    if (!text) {
        return;
    }
    while (*text) {
        uart_write_byte((uint8_t)*text++);
    }
}

static void uart_write_uint16(uint16_t value) {
    char buffer[6];
    int index = sizeof(buffer) - 1;
    buffer[index] = '\0';

    if (value == 0) {
        uart_write_byte('0');
        return;
    }

    while (value > 0 && index > 0) {
        index--;
        buffer[index] = (char)('0' + (value % 10));
        value /= 10;
    }

    uart_write_string(&buffer[index]);
}

static void uart_report_current(uint16_t pwm, MotorRetning retning) {
    if ((int)pwm == s_last_reported_pwm && retning == s_last_reported_dir) {
        return;
    }

    uart_init();
    uart_write_string("Current PWM: ");
    uart_write_uint16(pwm);
    uart_write_string(" Direction: ");
    uart_write_string(retning == MOTOR_RETNING_FREM ? "FWD" : "REV");
    uart_write_string("\r\n");

    s_last_reported_pwm = pwm;
    s_last_reported_dir = retning;
}

void motorSimulateISR(void) {
    motor_service_tick();
}

static void timer1_write_pwm(uint16_t pwm, MotorRetning retning) {
    if (!s_timer1_initialized) {
        timer1_pwm_init();
    }

    if (!(TCCR1A & (1 << COM1A1))) {
        TCCR1A = (TCCR1A & ~(1 << COM1A0)) | (1 << COM1A1);
    }

    OCR1A = pwm;

    if (retning == MOTOR_RETNING_FREM) {
        PORTB &= ~(1 << DIR_PIN);
    } else {
        PORTB |= (1 << DIR_PIN);
    }
}

static inline void motor_service_tick(void) {
    int current = g_current_pwm;
    int target = g_target_pwm;

    if (current < target) {
        current += PWM_SLEW_STEP;
        if (current > target) {
            current = target;
        }
    } else if (current > target) {
        current -= PWM_SLEW_STEP;
        if (current < target) {
            current = target;
        }
    }

    uint16_t pwm = clamp_pwm_to_10bit(current);
    g_current_pwm = pwm;
    timer1_write_pwm(pwm, g_motor_retning);
}

