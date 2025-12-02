#define F_CPU 16000000UL
#include <avr/io.h>
#include <stdio.h>
#include "uart.h"

#define BAUD 115200
#define MYUBRR F_CPU/16/BAUD-1

static int uart_putchar(char c, FILE *stream);

static FILE uart_output = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

void uartInit(void) {
    // Set baud rate
    UBRR0H = (unsigned char)(MYUBRR >> 8);
    UBRR0L = (unsigned char)MYUBRR;
    // Enable receiver and transmitter
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    // Set frame format: 8data, 2stop bit
    UCSR0C = (1 << USBS0) | (3 << UCSZ00);
    
    // Redirect stdout to UART
    stdout = &uart_output;
}

void uartSendByte(uint8_t byte) {
    // Wait for empty transmit buffer
    while (!(UCSR0A & (1 << UDRE0)));
    // Put data into buffer, sends the data
    UDR0 = byte;
}

void uartSendString(const char* str) {
    while (*str) {
        uartSendByte(*str++);
    }
}

static int uart_putchar(char c, FILE *stream) {
    if (c == '\n') {
        uartSendByte('\r');
    }
    uartSendByte(c);
    return 0;
}
