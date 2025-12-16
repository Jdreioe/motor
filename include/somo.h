#ifndef SOMO_H
#define SOMO_H

#include <stdint.h>

void somo_init(void);

void somo_play_track(uint8_t trackNum);

void somo_set_volume(uint8_t volume);

// Stop
void somo_stop(void);
// Specifikke lyde
void play_start_sound(void);
void play_refleks_sound(void);
void play_finish_sound(void);


void UART_Init(unsigned long baud, unsigned char databits, unsigned char rx_interrupt);
void UART_Send(char c);
void Volume_Up(void);
void Set_Volume(uint8_t volumeLevel);
void Play_Track(uint8_t trackNum);
void Play_Pause(void);
void Play_Stop(void);
void Next_Track(void);
void Previous_Track(void);

#endif // SOMO_H
