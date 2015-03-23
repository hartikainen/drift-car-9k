#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/delay.h>

#include "accelerate.h"
#include "output.h"
#include "bumper.h"
#include "stdio.h"

#define SCREEN_LOOP_COUNT 80000
#define STEERING_LOOP_COUNT 800
#define RPM_LOOP_COUNT 8000
#define BTN_LOOP_COUNT 40000
#define LAPTIME_LOOP_COUNT 12800
static volatile int str_timer_counter = 0;
static volatile int rpm_timer_counter = 0;   // remove at least one counter
static volatile unsigned int btn_timer_counter = 0;
static volatile unsigned int lap_timer_counter = 0;
static volatile int motor_pwm = 0;
static volatile char btn_delay = 0;

void setup_timer2(void)
{
  TCCR2B |= (1 << WGM22) | (0 << CS22) | (1 << CS21) | (0 << CS20);
  TIMSK2 |= (1 << OCIE2A);
  OCR2A = 0b11111111;
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
  char lapbuf[20];
  char recbuf[20];
  int i = 0;
  for(;;) {
    if (lap_timer_counter++ > LAPTIME_LOOP_COUNT) {
      update_laptime();
      lap_timer_counter = 0;
    }
    if (str_timer_counter++ > STEERING_LOOP_COUNT) {
      str_timer_counter = 0;
      read_bumper_turn_wheels();
    }
    if (rpm_timer_counter++ > RPM_LOOP_COUNT) {
      update_rpm();
      update_acceleration();
      rpm_timer_counter = 0;
    }
    if (btn_delay == 1){
      if (btn_timer_counter++ > BTN_LOOP_COUNT) {
        btn_delay = 0;
        PORTC = ~PORTC;
        btn_timer_counter = 0;
      }
    }
    if (!(PINE & 1 << PE5) && btn_delay == 0) {
      toggle_motor();
      btn_delay = 1;
      btn_timer_counter = 0;
    }

    if (i++ == SCREEN_LOOP_COUNT) {
      sprintf(rpmbuf, "RPM:     %d  ", get_rpm());
      output_string(rpmbuf,1,3);

      sprintf(bmpbuf, "BUMPER:  %d  ", get_bumper_int());
      output_string(bmpbuf, 1, 4);

      sprintf(pwmbuf, "PWM:     %d  ", get_pwm());
      output_string(pwmbuf, 1, 5);

      sprintf(pwmbuf, "tgt:     %d  ", get_target_rpm());
      output_string(pwmbuf, 1, 6);

      sprintf(lapbuf, "LAP: %d - %d.%d",
        get_current_lap(), get_laptime_secs(), get_laptime_partial());
      output_string(lapbuf, 1, 7);

      sprintf(lapbuf, "RECORD: %d - %d.%d",
        get_lap_record_lap(), get_lap_record_secs(), get_lap_record_partial());
      output_string(lapbuf, 1, 7);

      i = 0;
    }
  }
}

ISR(TIMER2_COMPA_vect) {

}
