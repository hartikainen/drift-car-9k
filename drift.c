#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/delay.h>

#include "accelerate.h"
#include "output.h"
#include "bumper.h"
#include "stdio.h"

#define SCREEN_LOOP_COUNT 10000
#define STEERING_LOOP_COUNT 100
#define RPM_LOOP_COUNT 1000
#define BTN_LOOP_COUNT 5000
#define LAPTIME_LOOP_COUNT 1600

static volatile unsigned int str_timer_counter = 0;
static volatile unsigned int rpm_timer_counter = 0;
static volatile unsigned int btn_timer_counter = 0;
static volatile unsigned int lap_timer_counter = 0;
static volatile unsigned int scr_timer_counter = 0;
static volatile char btn_delay = 0;

void setup_timer2(void)
{
  TCCR2B |= (1 << WGM22) | (0 << CS22) | (1 << CS21) | (0 << CS20); // timer with 8x prescaler
  TIMSK2 |= (1 << OCIE2A);
  OCR2A = 255;
}

void setup_leds(void) {
  DDRC = 0xff;
}

void setup_button(void)
{
  DDRE = 0x0;
}

void setup_tachometer(void) {
  DDRL &= ~(1<<PL2); // set the PL2 pin to input
  TCCR5B |= 1 << CS51 | 1 << CS52; // set the clock source to external clock source
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
  char strbuf[20];
  for(;;) {
    if (lap_timer_counter > LAPTIME_LOOP_COUNT) {
      update_laptime();
      lap_timer_counter = 0;
    }
    if (str_timer_counter > STEERING_LOOP_COUNT) {
      str_timer_counter = 0;
      read_bumper_turn_wheels();
    }
    if (rpm_timer_counter > RPM_LOOP_COUNT) {
      update_rpm();
      update_acceleration();
      rpm_timer_counter = 0;
    }
    if (btn_delay == 1){
      if (btn_timer_counter > BTN_LOOP_COUNT) {
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

    if (scr_timer_counter > SCREEN_LOOP_COUNT) {
      sprintf(strbuf, "RPM:     %d  ", get_rpm());
      output_string(strbuf,1,3);

      sprintf(strbuf, "BUMPER:  %d  ", get_bumper_int());
      output_string(strbuf, 1, 4);

      sprintf(strbuf, "PWM:     %d  ", get_pwm());
      output_string(strbuf, 1, 5);

      sprintf(strbuf, "tgt:     %d  ", get_target_rpm());
      output_string(strbuf, 1, 6);

      sprintf(strbuf, "LAP: %d - %d.%d",
        get_current_lap(), get_laptime_secs(), get_laptime_partial());
      output_string(strbuf, 1, 7);

      sprintf(strbuf, "RECORD: %d - %d.%d",
        get_lap_record_lap(), get_lap_record_secs(), get_lap_record_partial());
      output_string(strbuf, 1, 7);

      scr_timer_counter = 0;
    }
  }
}

ISR(TIMER2_COMPA_vect) {
  str_timer_counter++;
  rpm_timer_counter++;
  btn_timer_counter++;
  lap_timer_counter++;
  scr_timer_counter++;
}
