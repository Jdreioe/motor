#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stdio.h>

void uartInit(void);
void uartSendByte(uint8_t byte);
void uartSendString(const char* str);

#endif
