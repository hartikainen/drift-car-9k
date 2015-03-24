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

void setup_bumper_ddr(void) {
  BUMPER_DDR = 0;
}

#define Kp 0.01
#define Ki 0.000001
#define Kd 0.5

float integral_value = 0;
float derivate_value = 0;
static volatile int laptime_secs = 0;
static volatile int laptime_partial = 0;
static volatile int currentLap = 1;
static volatile int lap_record_secs = 0;
static volatile int lap_record_partial = 0;
static volatile int lap_record_lap = 0;

int get_laptime_secs(void) {
  return laptime_secs;
}
int get_laptime_partial(void) {
  return laptime_partial;
}
int get_current_lap(void) {
  return currentLap;
}
int get_lap_record_secs(void) {
  return lap_record_secs;
}
int get_lap_record_partial(void) {
  return lap_record_partial;
}
int get_lap_record_lap(void) {
  return lap_record_lap;
}


/* Returns the "rough" direction in the pwm units, */
/* between about WHEELS_MIN and WHEELS_MAX */
static float bumper_int = 0.0;
int target_from_bumper_led(uint8_t bumper_byte) {
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

int get_bumper_int(void) {
  return bumper_int;
}

int get_hamming_weight(uint8_t byte) {
  static const uint8_t NIBBLE_LOOKUP [16] =
  {
    0, 1, 1, 2, 1, 2, 2, 3,
    1, 2, 2, 3, 2, 3, 3, 4
  };
  return NIBBLE_LOOKUP[byte & 0x0F] + NIBBLE_LOOKUP[byte >> 4];
}

void update_laptime(void) {
  if (is_motor_on() && ++laptime_partial > 9) {
    laptime_secs++;
    laptime_partial = 0;
  }
}

void reset_laptime(void) {
  laptime_secs = 0;
  laptime_partial = 0;
}

void check_lap_record(void) {
  if (laptime_secs < lap_record_secs ||
    (laptime_secs == lap_record_secs && laptime_partial < lap_record_partial)) {
    lap_record_lap = currentLap;
    lap_record_secs = laptime_secs;
    lap_record_partial = laptime_partial;
  }
}

volatile int current_value = WHEELS_MIDDLE;

void read_bumper_turn_wheels(void) {
  int target_value, error, previous_error;
  float output;
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

  // Check if crossing finish line
  if (get_hamming_weight(bp) > 3 && laptime_secs > LAP_THRESHOLD) {
    check_lap_record();
    laptime_secs = 0;
    laptime_partial = 0;
    currentLap++;
  }

  // Turn the wheels.
  setup_pwm(current_value);
}
