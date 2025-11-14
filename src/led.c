 /****************************************
* "LED.C"                                  *
* Implementation file for "LED driver"     *
* Henning Hargaard                         *
****************************************/
#include <avr/io.h>
#define MAX_LED_NR 7
#define LED_MASK 0x0F  // Reserve lower four bits for LEDs; keep upper bits free for motor pins

void initLEDport()
{
  // Configure only LED bits as outputs to avoid clobbering motor pins on PB5/PB6
  DDRB = (DDRB & ~LED_MASK) | LED_MASK;
  PORTB &= ~LED_MASK;
}

void writeAllLEDs(unsigned char pattern)
{
  // Update only the LED bits, leave the motor pins untouched
  unsigned char masked = pattern & LED_MASK;
  PORTB = (PORTB & ~LED_MASK) | masked;
}

void turnOnLED(unsigned char led_nr)
{
  // Lokal variabel
  unsigned char mask;
  // Vi skal kun lave noget, hvis led_nr < 8
  if (led_nr <= MAX_LED_NR)
  {
    // Dan maske på basis af parameteren (led_nr)
    mask = 0b00000001 << led_nr;
    if (!(mask & LED_MASK))
      return;
    // Tænd den aktuelle lysdiode (de andre ændres ikke)
    PORTB |= mask;
  }
}

void turnOffLED(unsigned char led_nr)
{
  // Lokal variabel
  unsigned char mask;
  // Vi skal kun lave noget, hvis led_nr < 8
  if (led_nr <= MAX_LED_NR)
  {
    // Dan maske på basis af parameteren (led_nr)
    mask = 0b00000001 << led_nr;
    if (!(mask & LED_MASK))
      return;
    // Sluk den aktuelle lysdiode (de andre ændres ikke)
    PORTB &= ~mask;
  }
}

void toggleLED(unsigned char led_nr)
{
  // Lokal variabel
  unsigned char mask;
  // Vi skal kun lave noget, hvis led_nr < 8
  if (led_nr <= MAX_LED_NR)
  {
    // Dan maske på basis af parameteren (led_nr)
    mask = 0b00000001 << led_nr;
    if (!(mask & LED_MASK))
      return;
    // Toggle den aktuelle lysdiode (de andre ændres ikke)
    PORTB ^= mask;
  }
}