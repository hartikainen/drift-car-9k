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
  static int accel1[] = {130, 110, 0};
  static int accel2[] = {140, 130, 0};
  static int accel3[] = {160, 150, 0};

  switch (bp)
  {
    case 0b10000000:
    case 0b00000001:
      if (rpm > 5) {
        tgt = accel1[2];
      } else if (rpm > 2) {
        tgt = accel1[1];
      } else {
        tgt = accel1[0];
      }
      break;
    case 0b01000000:
    case 0b00000010:
      if (rpm > 5) {
        tgt = accel2[2];
      } else if (rpm > 2) {
        tgt = accel2[1];
      } else {
        tgt = accel2[0];
      }
      break;
    case 0b00100000:
    case 0b00000100:
      if (rpm > 6) {
        tgt = accel3[2];
      } else if (rpm > 3) {
        tgt = accel3[1];
      } else {
        tgt = accel3[0];
      }
      break;
   default:
      if (rpm < 3) {
        tgt = 130;
      }
      break;
  }
  return tgt;
}

#define PRED_SIZE_PER_RPM 5
#define MAX_RPM 15

#define STRAIGHT_PRED_SIZE 10
#define STEERING_PRED_SIZE 3

void get_predictions(uint8_t* ret_pred, int pred_size) {
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

int steering(uint8_t position) {
  return (position != STRAIGHT_STEERING);
}

#define STEEP_TURN_APPROACHING 1
#define RANDOM_TURN_IN_THE_MIDDLE 2

int get_turn_type(uint8_t* predictions, int pred_size) {
  int steering_count = get_steering_count(predictions, pred_size);

  if (steering_count > 1) {
    return STEEP_TURN_APPROACHING;
  } else if (predictions[pred_size-1] != STRAIGHT_STEERING) {
    return STEEP_TURN_APPROACHING;
  } else {
    return RANDOM_TURN_IN_THE_MIDDLE;
  }
  return RANDOM_TURN_IN_THE_MIDDLE;
}

int get_straights_ahead(uint8_t* predictions, int pred_size) {
  int count = 0;
  for (int i=0; i<pred_size; i++) {
    if (predictions[i] != STRAIGHT_STEERING) break;
    count++;
  }
  return count;
}

#define BRAKES_PER_RPM 3

int get_AI_target_pwm(void) {
  uint8_t cp = get_prediction(0);
  static int target_pwm = 0;

  int currently_steering = steering(cp);
  if (currently_steering) {
    target_pwm = get_target_rpm();
    return target_pwm;
  }

  int pred_size = PRED_SIZE_PER_RPM * rpm;
  if (!pred_size) pred_size = 5;
  uint8_t predictions[PRED_SIZE_PER_RPM * MAX_RPM];

  get_predictions(predictions, pred_size);
  int steering_count = get_steering_count(predictions, pred_size);

  if (steering_count == 0) {
    switch (rpm) {
    case (1):
      target_pwm = 190;
      break;
    case (2):
      target_pwm = 210;
      break;
    case (3):
      target_pwm = 240;
      break;
    case (4):
      target_pwm = 280;
      break;
    default:
      target_pwm = 300;
      break;
    }
  } else {
    int turn_type = get_turn_type(predictions, pred_size);
    int straights_ahead = get_straights_ahead(predictions, pred_size);

    if (turn_type == STEEP_TURN_APPROACHING) {
      int brakes_needed = BRAKES_PER_RPM * rpm;
      if (straights_ahead > brakes_needed + 5) {
	target_pwm = target_pwm;
      } else if (straights_ahead > brakes_needed) {
	target_pwm = 130;
      } else {
	target_pwm = -1;
      }
    } else if (turn_type == RANDOM_TURN_IN_THE_MIDDLE) {
      target_pwm = 130;
    } else {
      target_pwm = 130;
    }
  } 

  return target_pwm;
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

  int lap = get_current_lap();

  if (lap < 2) {
    setup_motor_pwm(110);
    return;
  } else {

    int target = get_AI_target_pwm();

    if (target < 0) {
      if (rpm > 3) {
	setup_brake();
	return;
      } else {
	target = 130;
      }
    }

    if (lap == 2) {
      target = (int)(0.9 * (float)target);
    } else if (lap == 3) {
      target = (int)(1.2 * (float)target);
    }

    setup_motor_pwm(target);
    return;
  }
}
