#include <avr/io.h>
#include <stdio.h>
#include "accelerate.h"
#include "bumper.h"
#include "output.h"

#define Kp 5.0
#define Ki 0.0
#define Kd 3.0
#define MAXPWM 300.0

static unsigned int rpm = 0;
static float pwm = 0.0;

void disable_motor_pwm(void) {
  TCCR4A |= ~_BV(COM4A1);
}

void setup_motor_pwm(int pwmoffset) {
  // Sets the pulse width modulation for the motor.
  PORTK |= 1 << PK0;
  DDRH |= 1 << PH3; // PH3 OC4A (Output Compare and PWM Output A for Timer/Counter4)
  TCCR4A |= 1 << WGM41 | 1 << COM4A1 | 1 << COM4A0;
  TCCR4B |= 1 << WGM43 | 1 << WGM42 | 1 << CS40;
  ICR4 = 800; // 20kHz without prescaler
  OCR4A = ICR4 - pwmoffset;
}

static char motor_on = 0;
void toggle_motor() {
  if (motor_on) {
    stop_motor();
    reset_laptime();
  } else {
    start_motor();
  }
}

void start_motor() {
  motor_on = 1;
}

void stop_motor() {
  motor_on = 0;
}

int get_rpm(void) {
  return rpm;
}

int get_pwm(void) {
  return (int)pwm;
}

int is_motor_on(void) {
  return motor_on;
}

void update_rpm(void) {
  static unsigned int previousTCNT = 0;
  if (TCNT5 < previousTCNT) {
    previousTCNT = 65536 - previousTCNT;
  }
  rpm = TCNT5 - previousTCNT;
  previousTCNT = TCNT5;
}

int get_target_rpm(void) {
  // calculate target rpm according to the bumper sensor
  // if we've driven straight for multiple intervals, increase target
  static int straight_counter = 0, tgt = 0;
  uint8_t bp = ~BUMPER_PIN;

  switch (bp)
  {
    case 0b10000000:
    case 0b00000001:
      tgt = 80;
      straight_counter = 0;
      break;
    case 0b01000000:
    case 0b00000010:
      tgt = 90;
      straight_counter = 0;
      break;
    case 0b00100000:
    case 0b00000100:
      tgt = 90;
      if (straight_counter > 1) {
	tgt = 0;
      }
      straight_counter = 0;
      break;
    case 0b00010000:
    case 0b00001000:
      if (straight_counter > 1) {
	tgt = 115;
      } else {
        tgt = 105;
      }
      straight_counter++;
      break;
    default:
      break;
  }
  return tgt;
}

void update_acceleration(void) {
  // PID control for the motor.
  // Calculates the error between rpm and target rpm
  // Calculates Proportional, Integral and Derivative values
  // which are used to calculate suitable pwm value.

  if (!motor_on) {
    setup_motor_pwm(0);
    return;
  }

  static float integral_value = 0.0;
  static float last_error = 0;
  float derivative_value = 0.0;
  float proportional_value = 0.0;
  float tf = (float)get_target_rpm();
  float error = tf - (float)rpm * 20.0;

  integral_value += error;
  derivative_value = error - last_error;
  proportional_value = error;
  pwm = 0.70*((Kp * proportional_value) + (Ki * integral_value) + (Kd * derivative_value));

  if (pwm > MAXPWM) pwm = MAXPWM;
  if (pwm < 0) pwm = 0;

  setup_motor_pwm((int)pwm);

  last_error = error;
}
