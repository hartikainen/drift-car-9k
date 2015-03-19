#include <avr/io.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include "bumper.h"
#include "output.h"

void setup_pwm(int val) {
  DDRB = 0xff;
  TCCR1A |= (1 << COM1A0) | (1 << COM1A1) | (1 << WGM11);
  TCCR1B |= (1 << CS10) | (1 << CS11) | (1 << WGM12) | (1 << WGM13);
  ICR1 = 4999;
  OCR1A = ICR1 - val;
}

void setup_bumper_ddr(void)
{
  BUMPER_DDR = 0;
}
  
#define Kp 0.01
#define Ki 0.000001
#define Kd 0.5

float integral_value = 0;
float derivate_value = 0;

/* Returns the "rough" direction in the pwm units, */
/* between about WHEELS_MIN and WHEELS_MAX */
float bumper_int = 0;
int target_from_bumper_led(uint8_t bumper_byte)
{
  switch (bumper_byte) {
  case 0b00000001: // OIKEA
    bumper_int = -4.0;
    break;
  case 0b00000010:
    bumper_int = -2.8;
    break;
  case 0b00000100:
    bumper_int = -1.3;
    break;
  case 0b00001000:
    bumper_int = -0.4;
    break;
  case 0b00010000:
    bumper_int = 0.4;
    break;
  case 0b00100000:
    bumper_int = 1.3;
    break;
  case 0b01000000:
    bumper_int = 2.8;
    break;
  case 0b10000000: // VASEN, WHEELS_MAXia vastava
    bumper_int = 4.0;
    break;
  default:
    bumper_int = bumper_int;
  }
  return (int)((float)WHEELS_MIDDLE + ((float)WHEELS_STEP * bumper_int));
}

volatile int current_value = WHEELS_MIDDLE;
volatile int counter_strike;
void read_bumper_turn_wheels(void)
{
  int target_value, error, previous_error;
  float output;
  uint8_t bp = ~BUMPER_PIN;
  target_value = target_from_bumper_led(bp);

  error = (target_value - current_value);

  integral_value += error;
  derivate_value = error - previous_error;
  previous_error = error;

  output = (Kp * (float)error) + (Ki * (float)integral_value) + (Kd * (float)derivate_value);
  /* Add or substract 0.5 to avoid flooring errors */
  /* Otherwise we end up in a situation where */
  /* current_value += 0 and we're off by up to 10 pwm units */
  if (output > 0) output += 1.0;
  if (output < 0) output -= 1.0;

  current_value += (int)(output);

  if (current_value > WHEELS_MAX) current_value = WHEELS_MAX;
  if (current_value < WHEELS_MIN) current_value = WHEELS_MIN;

  // Turn the wheels.
  setup_pwm(current_value);
}

