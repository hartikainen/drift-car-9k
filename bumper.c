#include <avr/io.h>
#include "bumper.h"

void setup_bumper_ddr(void) {
  BUMPER_DDR = 0;
}

void setup_bumper_wheel_timer(void) {
  TCCR2B |= (1 << WGM22) | (1 << CS22) | (1 << CS21) | (1 << CS20);
  TIMSK2 |= (1 << OCIE2A);
  OCR2A = 0b11111111;
}
