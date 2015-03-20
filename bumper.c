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

/* Returns the "rough" direction in the pwm units, */
/* between about WHEELS_MIN and WHEELS_MAX */
int target_from_bumper_led(uint8_t bumper_byte)
{
  static float bumper_float = 0;
  switch (bumper_byte) {
  case 0b00000001: // OIKEA
    bumper_float = -4.0;
    break;
  case 0b00000010:
    bumper_float = -2.8;
    break;
  case 0b00000100:
    bumper_float = -1.3;
    break;
  case 0b00001000:
    bumper_float = -0.4;
    break;
  case 0b00010000:
    bumper_float = 0.4;
    break;
  case 0b00100000:
    bumper_float = 1.3;
    break;
  case 0b01000000:
    bumper_float = 2.8;
    break;
  case 0b10000000: // VASEN, WHEELS_MAXia vastaava
    bumper_float = 4.0;
    break;
  default:
    bumper_float = bumper_float;
  }
  return (int)((float)WHEELS_MIDDLE + ((float)WHEELS_STEP * bumper_float));
}

void read_bumper_turn_wheels(void)
{
  static int current_value = (int)WHEELS_MIDDLE;
  static int previous_error = 0;
  static float integral_value = 0.0;
  int target_value, error;
  float output, derivate_value;
  uint8_t bp = ~BUMPER_PIN;

  target_value = target_from_bumper_led(bp);

  error = (target_value - current_value);

  integral_value += error;
  derivate_value = error - previous_error;
  previous_error = error;

  output = (Kp * (float)error) + (Ki * (float)integral_value) + (Kd * (float)derivate_value);
  /* Add or substract 1.0 to avoid flooring errors */
  /* Otherwise we end up in a situation where */
  /* current_value += 0 and we're off by up to 9 pwm units */
  if (output > 0) output += 1.0;
  if (output < 0) output -= 1.0;

  current_value += (int)(output);

  if (current_value > WHEELS_MAX) current_value = WHEELS_MAX;
  if (current_value < WHEELS_MIN) current_value = WHEELS_MIN;

  // Turn the wheels.
  setup_pwm(current_value);
}

