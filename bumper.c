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
  
#define Kp 0.8
#define Ki 0.001
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

/* Returns the "rough" direction in the pwm units, */
/* between about 300 and 460 */
int bumper_int = 0;
int target_from_bumper_led(uint8_t bumper_byte)
{
  switch (bumper_byte) {
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
    break;
  default:
    bumper_int = bumper_int;
  }
  return (int)WHEELS_MIDDLE + ((int)WHEELS_STEP * bumper_int);
}

volatile int current_value = WHEELS_MIDDLE;
volatile int counter_strike;
void read_bumper_turn_wheels(void)
{
  int target_value, y,  pide = 0;
  uint8_t bp = ~BUMPER_PIN;
  target_value = target_from_bumper_led(bp);

  pide = (target_value - current_value);

  integral_value += pide;
  derivate_value = pide - previous;
  previous = pide;

  y = (int)(Kp * (float)pide) + (int)(Ki * (float)integral_value) + (Kd * (float)derivate_value);
  current_value += y;//= (int)WHEELS_MIDDLE + y;
  if (current_value > WHEELS_MAX) current_value = WHEELS_MAX;
  if (current_value < WHEELS_MIN) current_value = WHEELS_MIN;

  // Turn the wheels.
  setup_pwm(current_value);

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

