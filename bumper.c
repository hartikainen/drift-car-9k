#include <avr/io.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include "bumper.h"
#include "output.h"
#include "accelerate.h"
#include "drift.h"

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

int oot_counter = 0; // out of track counter

static int laps[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
static int odo = 0;
static volatile int laptime_secs = 0;
static volatile int laptime_partial = 0;
static volatile int currentLap = 1;
static volatile int lap_record_secs = 0;
static volatile int lap_record_partial = 0;
static volatile int lap_record_lap = 0;
static float bumper_float = 0.0;

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

int get_bumper_float(void) {
  return bumper_float;
}

/* Hamming weight, e.g. the count of active bumper leds */
int get_hamming_weight(uint8_t byte) {
  static const uint8_t NIBBLE_LOOKUP [16] = {
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
  if (lap_record_lap == 0 || laptime_secs < lap_record_secs ||
    (laptime_secs == lap_record_secs && laptime_partial < lap_record_partial)) {
    lap_record_lap = currentLap;
    lap_record_secs = laptime_secs;
    lap_record_partial = laptime_partial;
  }
}

float BUMPER_FLOAT_STRAIGHT[8] = {-0.9, -0.5, -0.3, -0.0, 0.3, 0.5, 0.9, 1.4};
float BUMPER_FLOAT_TURNING[8] = {-4.0, -3.0, -2.0, -1.0, 1.0, 2.0, 3.0, 4.0};
float BUMPER_FLOAT_FIRST_LAP[8] = {-4.0, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 4.0};
/* Returns the target direction in the pwm units, */
/* between about WHEELS_MIN and WHEELS_MAX */
int target_from_bumper_led(uint8_t bumper_byte) {
  static int idx = 0;
  int next_direction;

  switch (bumper_byte) {
  case 0b00000001: // RIGHT
    idx = 0;//-4.0;
    break;
  case 0b00000010:
    idx = 1;//-2.8;
    break;
  case 0b00000100:
    idx = 2;//-2.0;
    break;
  case 0b00001000:
    idx = 3;//-0.5;
    break;
  case 0b00010000:
    idx = 4;//0.5;
    break;
  case 0b00100000:
    idx = 5;//2.0;
    break;
  case 0b01000000:
    idx = 6;//2.8;
    break;
  case 0b10000000: // LEFT, corresponds to WHEELS_MAX
    idx = 7;//4.0;
    break;
  default:
    /* Case where there is no tape under the bumper led, */
    /* or when several leds are on at the same time */
    /* Stop the car if it's been out of the track for about 0.5 seconds */
    break;
  }

  next_direction = get_prediction(1);

  if (currentLap > 1) {
    switch (next_direction) {
    case (LEFT_STEERING):
    case (RIGHT_STEERING):
      bumper_float = BUMPER_FLOAT_TURNING[idx];
      break;
    case (STRAIGHT_STEERING):
      bumper_float = BUMPER_FLOAT_STRAIGHT[idx];
      break;
    default:
      bumper_float = BUMPER_FLOAT_STRAIGHT[idx];
      break;
    }
  } else {
    bumper_float = BUMPER_FLOAT_FIRST_LAP[idx];

    uint8_t prediction = get_prediction(4);
  }

  return (int)((float)WHEELS_MIDDLE + ((float)WHEELS_STEP * bumper_float));
}

void check_finish_line(void) {
// Check if crossing finish line
  uint8_t bp = ~BUMPER_PIN;
  if (get_hamming_weight(bp) > 3 && laptime_secs > LAP_THRESHOLD) {
    check_lap_record();
    laptime_secs = 0;
    laptime_partial = 0;
    laps[currentLap] = TCNT5 - odo;
    odo += laps[currentLap];
    currentLap++;
    TCNT5 = 0;
  }
}

int *get_laps(void) {
  return laps;
}

/* Function for turning the wheels according to the bumper leds reading */
/* Basically, bumper led show where we should turn, and the function turns */
/* The wheels according to the PID control values */
void read_bumper_turn_wheels(uint8_t bp) {
  static int integral_value = 0, current_value = WHEELS_MIDDLE;
  int target_value, error, previous_error, old_value;
  float output, derivate_value;
  target_value = target_from_bumper_led(bp);

  old_value = current_value;
  error = (target_value - current_value);

  integral_value += error;
  derivate_value = error - previous_error;
  previous_error = error;

  output = (sKp * (float)error) + (sKi * (float)integral_value) + (sKd * (float)derivate_value);
  /* Add or substract 1.0 to avoid flooring errors */
  /* Otherwise we end up in a situation where */
  /* current_value += 0 and we're off by up to 9 pwm units */
  if (output > 0) output += 1.0;
  if (output < 0) output -= 1.0;

  current_value += (int)(output);

  /* Limit the values s.t. the servo won't be turned too much */
  if (current_value > WHEELS_MAX) current_value = WHEELS_MAX;
  if (current_value < WHEELS_MIN) current_value = WHEELS_MIN;

  // Try to detect overstreering when/before crossing finish line
  if (current_value == WHEELS_MAX && current_value - old_value > WHEELS_STEP) {
    current_value = old_value;
  }
  else if (current_value == WHEELS_MIN && old_value - current_value > WHEELS_STEP) {
    current_value = old_value;
  }

  // Turn the wheels.
  setup_pwm(current_value);
}
