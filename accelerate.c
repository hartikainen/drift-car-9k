#include <avr/io.h>
#include <stdio.h>
#include "accelerate.h"
#include "bumper.h"
#include "output.h"
#include "drift.h"

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
  PORTK = 1 << PK0;
  DDRH |= 1 << PH3; // PH3 OC4A (Output Compare and PWM Output A for Timer/Counter4)
  TCCR4A |= 1 << WGM41 | 1 << COM4A1 | 1 << COM4A0;
  TCCR4B |= 1 << WGM43 | 1 << WGM42 | 1 << CS40;
  ICR4 = 800; // 20kHz without prescaler
  OCR4A = ICR4 - pwmoffset;
}

void setup_brake(void) {
  // Sets the pulse width modulation for the motor.
  PORTK |= (1 << PK1) | (1 << PK0) ;
  DDRH |= 1 << PH3; // PH3 OC4A (Output Compare and PWM Output A for Timer/Counter4)
  TCCR4A |= 1 << WGM41 | 1 << COM4A1 | 1 << COM4A0;
  TCCR4B |= 1 << WGM43 | 1 << WGM42 | 1 << CS40;
  ICR4 = 800; // 20kHz without prescaler
  OCR4A = ICR4 - 20;
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
  static int tgt = 0;
  uint8_t bp = ~BUMPER_PIN;
  static int accel1[] = {120, 100, 0};
  static int accel2[] = {140, 120, 0};
  static int accel3[] = {160, 120, 0};
  static int accel4[] = {200, 180, 160, 0};

  switch (bp)
  {
    case 0b10000000:
    case 0b00000001:
      if (rpm > 4) {
        tgt = accel1[2];
      } else if (rpm > 2) {
        tgt = accel1[1];
      } else {
        tgt = accel1[0];
      }
      break;
    case 0b01000000:
    case 0b00000010:
      if (rpm > 4) {
        tgt = accel2[2];
      } else if (rpm > 2) {
        tgt = accel2[1];
      } else {
        tgt = accel2[0];
      }
      break;
    case 0b00100000:
    case 0b00000100:
      if (rpm > 4) {
        tgt = accel3[2];
      } else if (rpm > 2) {
        tgt = accel3[1];
      } else {
        tgt = accel3[0];
      }
      break;
    case 0b00010000:
    case 0b00001000:
      if (rpm > 4) {
        tgt = accel4[2];
      } else if (rpm > 2) {
        tgt = accel4[1];
      } else {
        tgt = accel4[0];
      }
     break;
    default:
      break;
  }
  return tgt;
}

#define STRAIGHT_PRED_SIZE 10
#define STEERING_PRED_SIZE 3

void get_predictions(int pred_size, uint8_t* ret_pred) {
  for (int i=0; i<pred_size; i++) {
    ret_pred[i] = get_prediction(i+1);
  }
}

int get_steering_count(uint8_t* preds, int pred_size) {
  int count = 0;
  for (int i=0; i<pred_size; i++) {
    if (preds[i] == LEFT_STEERING || preds[i] == RIGHT_STEERING) {
      count++;
    }
  }
  return count;
}

int get_AI_target_pwm(void) {
  uint8_t cp = get_prediction(0);
  int target_pwm = 0;

  if (cp == STRAIGHT_STEERING) {
    uint8_t predictions[STEERING_PRED_SIZE];
    get_predictions(STEERING_PRED_SIZE, predictions);
    int steering_count = get_steering_count(predictions, STEERING_PRED_SIZE);
    if (predictions[STEERING_PRED_SIZE-1] == LEFT_STEERING || predictions[STEERING_PRED_SIZE-1] == RIGHT_STEERING) {
      target_pwm = -1;
    } else if (steering_count > 5) {
      target_pwm = 110;
    } else if (steering_count == 0) {
      target_pwm = 220;
    } else {
      target_pwm = 110;
    }
  } else {
    uint8_t predictions[STRAIGHT_PRED_SIZE];
    get_predictions(STRAIGHT_PRED_SIZE, predictions);
    int steering_count = get_steering_count(predictions, STRAIGHT_PRED_SIZE);
    int straight_count = STRAIGHT_PRED_SIZE - steering_count;
    target_pwm = 110;
  }

  return target_pwm;
}

int get_actual_pwm(int target) {
  if (target > 150) {
    PORTC = 0b11111111;
    if (rpm < 5) {
      return 180;
    } else {
      return 210;
    }
  } else {
    PORTC = 0;
    if (rpm > 2) {
      return -1;
    } else {
      return 130;
    }
  }
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

//  static float integral_value = 0.0;
//  static float last_error = 0;
//  float derivative_value = 0.0;
//  float proportional_value = 0.0;
//  float tf = (float)get_target_rpm();
//  float error = tf - (float)rpm * 20.0;

  int lap = get_current_lap();

  if (lap < 2) {
    setup_motor_pwm(110);
    return;
  } else {
    PORTC = ~PORTC;
    int target = get_AI_target_pwm();
    int actual = get_actual_pwm(target);

    if (actual < 0 || target < 0) {
      if (rpm > 2) {
	setup_brake();
	return;
      } else {
	actual = 110;
      }
    }
    setup_motor_pwm(actual);
    return;
  }

//  integral_value += error;
//  derivative_value = error - last_error;
//  proportional_value = error;
//  pwm = 0.70*((Kp * proportional_value) + (Ki * integral_value) + (Kd * derivative_value));

//  if (pwm > MAXPWM) pwm = MAXPWM;
//  if (pwm < 0) pwm = 0;

//  setup_motor_pwm((int)pwm);
//  setup_motor_pwm(get_target_rpm());
//  setup_motor_pwm(120);

//  last_error = error;
}
