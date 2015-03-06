#include <avr/io.h>
#include "bumper.h"

volatile int steering_locked = 0;
uint8_t bumper;

void setup_bumper_ddr(void)
{
  BUMPER_DDR = 0;
}

void setup_bumper_timer(void)
{
  TCCR2B |= (1 << WGM22) | (1 << CS22) | (1 << CS21) | (1 << CS20);
  TIMSK2 |= (1 << OCIE2A);
  OCR2A = 0b11111111;
}

void release_steering(void)
{
  steering_locked = 0;
  TCCR1A &= ~_BV(COM1A1); // 
}

void reset_bumper_timer(void)
{
  TCNT2 = 0;
}

void read_bumper_turn_wheels(void)
{
  bumper = ~BUMPER_PIN;
  if (!steering_locked) {
    steering_locked = 1;
    reset_bumper_timer();

    switch (bumper) {
    case 0b00000001:
      setup_pwm(WHEELS_MIDDLE - 40);
      break;
    case 0b00000010:
      setup_pwm(WHEELS_MIDDLE - 30);
      break;
    case 0b00000100:
      setup_pwm(WHEELS_MIDDLE - 20);
      break;
    case 0b00001000:
      setup_pwm(WHEELS_MIDDLE - 10);
      break;
    case 0b00010000:
      setup_pwm(WHEELS_MIDDLE + 10);
      break;
    case 0b00100000:
      setup_pwm(WHEELS_MIDDLE + 20);
      break;
    case 0b01000000:
      setup_pwm(WHEELS_MIDDLE + 30);
      break;
    case 0b10000000:
      setup_pwm(WHEELS_MIDDLE + 40);
      break;
    default:
      setup_pwm(WHEELS_MIDDLE);
    }
  }
}
