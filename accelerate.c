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

#define Kp 1.0
#define Ki 0.5
#define Kd 2.0
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
  static int rpm = 0;
  uint8_t bp = ~BUMPER_PIN;
  if ((bp == 0x10000000) || (bp == 0x00000001)) {
    rpm = 2;
  } else if ((bp == 0x01000000) || (bp == 0x00000010)) {
    rpm = 2;
  } else if ((bp == 0x00100000) || (bp == 0x00000100)) {
    rpm = 3;
  } else if ((bp == 0x00010000) || (bp == 0x00001000)) {
    rpm = 4;
  }
  return rpm;
}

void update_acceleration(void) {
  static float integral_value = 0.0;
  static unsigned int last_rpm = 0;
  float derivative_value = 0.0;
  float proportional_value = 0.0;
  float tf = (float)target;
  int target = get_target_rpm();

  if (motor_on) {
    integral_value += tf - (float)rpm; 
    derivative_value = (float)rpm - (float)last_rpm;
    proportional_value = tf - (float)rpm;
    pwm = 10.0*((Kp * proportional_value) + (Ki * integral_value) + (Kd * derivative_value));
    if (pwm > MAXPWM) {
      pwm = MAXPWM;
    }
    setup_motor_pwm((int)pwm);
  } 
  last_rpm = rpm;
  if (!motor_on) {
    setup_motor_pwm(0);
  }
}
