#include <avr/io.h>
#include <stdio.h>
#include "accelerate.h"
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
#define Ki 1.0
#define Kd 0.0
#define MAXPWM 200.0

static unsigned int rpm = 0;
int count = 0;

void update_acceleration(int target) {
  static float integral_value = 0.0;
  static float derivative_value = 0.0;
  static unsigned int previousTCNT = 0;
  static int previous = 0;
  float tf = ((float)target/100.0) * MAXPWM;

  float pwm = 0.0;
  if (TCNT5 < previousTCNT) {
    previousTCNT = 65536 - previousTCNT;
  }
  rpm = TCNT5 - previousTCNT;
  if (count++ > 100)
  {
    char str[20];
    count = 0;
 
    previousTCNT = TCNT5;
    if (motor_on) {
      integral_value += tf - rpm; 
      derivative_value = tf - previous;
      pwm = (Kp * (float)tf) + (Ki * integral_value) + (Kd * derivative_value);
      if (pwm > MAXPWM) {
        pwm = MAXPWM;
      }
      setup_motor_pwm((int)pwm); 
    } 
    sprintf(str, "rpm? %d %d %d    ", rpm, (int)pwm, (int)integral_value);
    output_string(str, 1, 5);
  }
  if (!motor_on) {
    setup_motor_pwm(0);
  }
}
