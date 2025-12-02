/************************************
* "Switch.c"                           *
* Implementation for SWITCH driver. *
* Henning Hargaard                     *
************************************/
#include <avr/io.h>

// Klargør switch-porten
void initSwitchPort()
{
  // Sæt PORTA som input
  DDRA = 0x00;   // Alle bits i PORTA som input
  PORTA = 0xff; 
}

// Læser alle switches samtidigt
unsigned char switchStatus()
{
  return ~PINA; // Returner status af PORTA
}

// Returnerer TRUE, hvis switchen med nummeret
// "switch_nr" er aktiveret - ellers returneres FALSE
unsigned char switchOn(unsigned char switch_nr)
{
  if (switch_nr < 8) 
  {
    return ((~PINA) & (1 << switch_nr)) ? 1 : 0; // Aktive lave: TRUE når kontakt er trykket
  }
  return 0; // Returner FALSE (0) hvis switch_nr er ugyldig
}