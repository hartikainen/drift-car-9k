#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/delay.h>

#include "output.h"
#include "bumper.h"
#include "PID.h"
#include "stdio.h"

void setup_motor_pwm(int pwmoffset) {
  PORTK |= 1 << PK0;
  DDRH |= 1 << PH3; // PH3 OC4A (Output Compare and PWM Output A for Timer/Counter4)
  TCCR4A |= 1 << WGM41 | 1 << COM4A1 | 1 << COM4A0;
  TCCR4B |= 1 << WGM43 | 1 << WGM42 | 1 << CS40;
  ICR4 = 800; // 20kHz without prescaler
  OCR4A = ICR4 - pwmoffset;
}

void setup_timer2(void)
{
  TCCR2B |= (1 << WGM22) | (0 << CS22) | (1 << CS21) | (0 << CS20);
  TIMSK2 |= (1 << OCIE2A);
  OCR2A = 0b11111111;
}

void disable_motor_pwm(void) {
  TCCR4A |= ~_BV(COM4A1);
}

void setup_leds(void) {
  DDRC = 0xff;
}

void setup_button(void)
{
  DDRE = 0x0; // 0 or FF?
}

void setup_tachometer(void) {
  DDRL &= ~(1<<PL2);
  TCCR5B |= 1 << CS51 | 1 << CS52;
}

volatile char str_timer_counter = 0;
volatile char rpm_timer_counter = 0;   // remove at least one counter
volatile unsigned int last_rpm = 0;
volatile unsigned int rpm = 0;
volatile int bumper = 0;
volatile int motor_pwm = 0;
const int STARTING_SPEED = 160;

int main(void)
{

  setup_leds();
  setup_tachometer();
  setup_bumper_ddr();
  setup_timer2();

  USART_init(MYUBRR);
  PORTC = 0;

  setup_button();

  sei();

  char bmpbuf[20];
  char rpmbuf[20];
  char pwmbuf[20];

  setup_motor_pwm(STARTING_SPEED);
  for(;;) {
    if (!(PINE & 1 << PE5)) {
      motor_pwm = motor_pwm == 0 ? STARTING_SPEED : 0;
      setup_motor_pwm(motor_pwm);
      //      reset_PID_stuff();
    }
    sprintf(rpmbuf, "RPM:     %d  ", rpm);
    output_string(rpmbuf,1,3);

    sprintf(bmpbuf, "BUMPER:  %d  ", bumper);
    output_string(bmpbuf, 1, 4);

    sprintf(pwmbuf, "PWM:     %d  ", motor_pwm);
    output_string(pwmbuf, 1, 5);
  }
}

#define STEERING_LOOP_COUNT 1
#define RPM_LOOP_COUNT 50

ISR(TIMER2_COMPA_vect) {
  PORTC = ~PORTC; // TEST, blink the leds

  if (str_timer_counter++ > STEERING_LOOP_COUNT) {
    str_timer_counter = 0;
    bumper = read_bumper_turn_wheels();
  }
  
  if (rpm_timer_counter++ > RPM_LOOP_COUNT) {
    rpm_timer_counter = 0;
    rpm = TCNT5-last_rpm; // weird rpm when TCNT5 overflows

    last_rpm = TCNT5;
  }
}

