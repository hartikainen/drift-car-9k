#include <avr/io.h>
#include <inttypes.h>
#include <stdio.h>
#include "bumper.h"
#include "output.h"

volatile int steering_locked = 0;
int bumper_int = 0;
int pide, pwm_val;
uint8_t bumper;

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

volatile int counter = 0;
void turn_wheels(float direction) {
  pwm_val = (int) (WHEELS_MIDDLE + (WHEELS_STEP * direction));
  if (pwm_val > WHEELS_MAX) pwm_val = WHEELS_MAX;
  if (pwm_val < WHEELS_MIN) pwm_val = WHEELS_MIN;

  /* if (counter++ > 15) { */
  /*   counter = 0; */
  /*   char bumpstr[10]; */
  /*   itoa((int)pwm_val, bumpstr, 10); */
  /*   output_string(bumpstr, 2, 2); */
  /* } */

  setup_pwm(pwm_val);
}

#define Kp 0.2
#define Ki 0.1
#define Kd 5
#define TARGET 0.0

float integral_value = 0;
float derivate_value = 0;
int previous = 0;

void reset_PID_stuff(void) {
  return;
  /* integral_value = 0; */
  /* derivate_value = 0; */
  /* fake_Ki += 0.2; */

  /* char kistr[10]; */
  /* float paskas = 0.5; */
  /* snprintf(kistr, 10, "a%lfb", paskas); */

  /* output_string(kistr,4,4); */
}
void read_bumper_turn_wheels(void)
{
  float y;
  uint8_t bp = ~BUMPER_PIN;
  bumper = bp;
  switch (bumper) {
  case 0b00000001:
    bumper_int = -4;
    break;
  case 0b00000010:
    bumper_int = -3;
    break;
  case 0b00000100:
    bumper_int = -2;
    break;
  case 0b00001000:
    if (bumper_int == 1 || bumper_int == 0) {
      bumper_int = 0;
      break;
    }
    bumper_int = -1;
    break;
  case 0b00010000:
    if (bumper_int == -1 || bumper_int == 0) {
      bumper_int = 0;
      break;
    }
    bumper_int = 1;
    break;
  case 0b00100000:
    bumper_int = 2;
    break;
  case 0b01000000:
    bumper_int = 3;
    break;
  case 0b10000000:
    bumper_int = 4;
    break;
  default:
    bumper_int = bumper_int;
  }

  pide = (float)bumper_int;
  integral_value += (float)bumper_int;
  derivate_value = bumper_int - previous;
  previous = bumper_int;

  if (integral_value > 2) {
    integral_value = 2;
  }
  if (integral_value < -2) {
    integral_value = -2;
  }

  y = (Kp * pide) + (Ki * integral_value) + (Kd * derivate_value);

  turn_wheels(y);
}

