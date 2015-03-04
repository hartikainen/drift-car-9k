#include <inttypes.h> /* definitions for uint8_t and others */
#include <avr/io.h>   /* definitions for all PORT* and other registers. You absolutely will need this one */
#include <avr/interrupt.h>

// TODO: remove this
#include <util/delay.h>

#define LEDS_PORT PORTC
#define LEDS_DDR  DDRC
#define LEDS_PIN  PINC

#define SWITCH_PORT PORTA
#define SWITCH_DDR DDRA
#define SWITCH_PIN PINA

#define BLINK_LED PC2
#define OTHER_LED PC0

int main(void) {
  for(;;) {
  }
}
