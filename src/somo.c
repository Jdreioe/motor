#include "somo.h"
#include <avr/io.h>

// Hjælpefunktion der sender package
static void somo_send_packet(uint8_t cmd, uint8_t param1, uint8_t param2) {
    uint16_t checksum = 0xFFFF - (cmd + param1 + param2) + 1; 
    uint8_t buffer[8] = {0x7E, cmd, 0x00, param1, param2, (uint8_t)(checksum >> 8), (uint8_t)(checksum & 0xFF), 0xEF};

    for (int i = 0; i < 8; i++) {
        while (!(UCSR3A & (1 << UDRE3))); // Vent til empty buffer før næste 
        UDR3 = buffer[i];
    }
}

// Initialize UART3 for SOMO module (9600 baud)
void somo_init(void) {
    // 9600 baud @ 16MHz: UBRR = (16000000 / (16 * 9600)) - 1 = 103
    unsigned int ubrr = 103;
    UBRR3H = (unsigned char)(ubrr >> 8);
    UBRR3L = (unsigned char)ubrr;
    
    UCSR3B = (1 << TXEN3);                  // Enable Transmitter
    UCSR3C = (1 << UCSZ31) | (1 << UCSZ30); // 8 data bits, 1 stop bit, no parity

    somo_set_volume(25);
}

void somo_play_track(uint8_t trackNum) {
    somo_send_packet(0x03, 0x00, trackNum);
}

void somo_set_volume(uint8_t volume) {
    if (volume > 30) volume = 30;
    somo_send_packet(0x06, 0x00, volume);
}

void somo_stop(void) {
    somo_send_packet(0x16, 0x00, 0x00);
}
void play_start_sound(void) {
    somo_play_track(0x00);
}
void play_refleks_sound(void) {
    somo_play_track(0x01);
}
void play_finish_sound(void) {
    somo_play_track(0x02);
}