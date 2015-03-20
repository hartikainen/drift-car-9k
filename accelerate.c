#include <avr/io.h>
#include <stdio.h>
#include "accelerate.h"
#include "bumper.h"
#include "output.h"

void disable_motor_pwm(void) {
  TCCR4A |= ~_BV(COM4A1);
}

void setup_motor_pwm(int pwmoffset) {
  PORTK |= 1 << PK0;
  DDRH |= 1 << PH3; // PH3 OC4A (Output Compare and PWM Output A for Timer/Counter4)
  TCCR4A |= 1 << WGM41 | 1 << COM4A1 | 1 << COM4A0;
  TCCR4B |= 1 << WGM43 | 1 << WGM42 | 1 << CS40;
  ICR4 = 800; // 20kHz without prescaler
  OCR4A = ICR4 - pwmoffset;
}

static char motor_on = 0;
void toggle_motor() {
  motor_on = !motor_on;
}

#define Kp 2.0
#define Ki 0.1
#define Kd 4.0
#define MAXPWM 300.0

static unsigned int rpm = 0;
int count = 0;

int get_rpm(void) {
  return rpm;
}

static float pwm = 0.0;

int get_pwm(void) {
  return (int)pwm;
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
  static int tgt = 0;
  uint8_t bp = ~BUMPER_PIN;
  switch (bp)
  {
    case 0b10000000:
    case 0b01000000:
    case 0b00000010:
    case 0b00000001:
      tgt = 0;
      break;
    case 0b00100000:
    case 0b00000100:
      tgt = 1;
      break;
    case 0b00010000:
    case 0b00001000:
      tgt = 4;
      break;
    default:
      break;
  }
  return tgt;
}

void update_acceleration(void) {
  static float integral_value = 0.0;
  static float last_error = 0;
  float derivative_value = 0.0;
  float proportional_value = 0.0;
  int target = get_target_rpm();
  float tf = (float)target;
  float error = tf - (float)rpm;

  if (motor_on) {

    integral_value += error; 
    derivative_value = error - last_error;
    proportional_value = error;
    pwm = 10.0*((Kp * proportional_value) + (Ki * integral_value) + (Kd * derivative_value));
    if (pwm > MAXPWM) {
      pwm = MAXPWM;
    }
    setup_motor_pwm((int)pwm);
  } 
  last_error = error;
  if (!motor_on) {
    setup_motor_pwm(0);
  }
}
