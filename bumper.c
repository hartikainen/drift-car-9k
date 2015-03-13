#include <avr/io.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include "bumper.h"
#include "output.h"

volatile int steering_locked = 0;
int bumper_int = 0;
int pwm_val;
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

void turn_wheels(int direction) {
  //  pwm_val = (int) (WHEELS_MIDDLE + (WHEELS_STEP * direction));
  pwm_val = direction;
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

#define Kp 0.01
#define Ki 0.002
#define Kd 0

float integral_value = 0;
float derivate_value = 0;
int previous = 0;

float fake_Ki = Ki;
void reset_PID_stuff(void) {
  fake_Ki += 0.001;
  char fakekistr[10];
  sprintf(fakekistr, "fake_Ki: %d", (int)fake_Ki);
  output_string(fakekistr,1,1);
}

volatile int current_value = WHEELS_MIDDLE;
volatile int counter_strike;
void read_bumper_turn_wheels(void)
{
  int target_value, pide = 0, y;
  uint8_t bp = ~BUMPER_PIN;
  bumper = bp;
  switch (bumper) {
  case 0b00000001: // OIKEA
    bumper_int = -4;
    break;
  case 0b00000010:
    bumper_int = -3;
    break;
  case 0b00000100:
    bumper_int = -2;
    break;
  case 0b00001000:
    bumper_int = -1;
    break;
  case 0b00010000:
    bumper_int = 1;
    break;
  case 0b00100000:
    bumper_int = 2;
    break;
  case 0b01000000:
    bumper_int = 3;
    break;
  case 0b10000000: // VASEN, WHEELS_MAXia vastava
    bumper_int = 4;
    PORTC = ~PORTC; // TEST, blink the leds
    break;
  default:
    bumper_int = bumper_int;
  }

  target_value = (int)WHEELS_MIDDLE + ((int)WHEELS_STEP * bumper_int);
  pide = (target_value - current_value);

  integral_value += pide;
  derivate_value = pide - previous;
  previous = pide;

  y = (int)(Kp * (float)pide) + (int)(fake_Ki * (float)integral_value) + (Kd * derivate_value);
  current_value = (int)WHEELS_MIDDLE + y;
  turn_wheels(current_value);

  /* if (counter_strike++ > 200) { */
  /*   counter_strike = 0; */
  /*   char ystr[10]; */
  /*   char currentstr[10]; */
  /*   char targetstr[10]; */
  /*   char wmidstr[10]; */
  /*   char wstepstr[10]; */

  /*   sprintf(ystr, "y: %d\n", y); */
  /*   sprintf(currentstr, "cv: %d", current_value); */
  /*   sprintf(targetstr, "tv: %d", target_value); */
  /*   sprintf(wmidstr, "W_MID: %d", WHEELS_MIDDLE); */
  /*   sprintf(wstepstr, "W_STP: %d", WHEELS_STEP); */

  /*   output_string(ystr,1,1); */
  /*   output_string(currentstr, 1, 2); */
  /*   output_string(targetstr,1,3); */
  /*   output_string(wmidstr,1,4); */
  /*   output_string(wstepstr,1,5); */
  /* } */
}

