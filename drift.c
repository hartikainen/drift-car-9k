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
#define FINISHLINE_LOOP_COUNT 100
#define RPM_LOOP_COUNT 1000
#define BTN_LOOP_COUNT 5000
#define LAPTIME_LOOP_COUNT 781

static volatile unsigned int str_timer_counter = 0;
static volatile unsigned int rpm_timer_counter = 0;
static volatile unsigned int btn_timer_counter = 0;
static volatile unsigned int lap_timer_counter = 0;
static volatile unsigned int scr_timer_counter = 0;
static volatile unsigned int finish_line_counter = 0;

static volatile char btn_delay = 0;

void setup_timer2(void) {
  // setup timer2 with 8x prescaler and overflow interrupt
  TCCR2B |= (1 << WGM22) | (0 << CS22) | (1 << CS21) | (0 << CS20);
  TIMSK2 |= (1 << OCIE2A);
  OCR2A = 255;
}

void setup_leds(void) {
  PORTC = 0;
  DDRC = 0xff;
}

void setup_button(void) {
  DDRE = 0x0;
}

void setup_tachometer(void) {
  // Set the tachometer pin as input. Set clock 5 to external clock source.
  DDRL &= ~(1<<PL2);
  TCCR5B |= 1 << CS51 | 1 << CS52;
}

int main(void) {
  setup_leds();
  setup_tachometer();
  setup_bumper_ddr();
  setup_timer2();

  USART_init(MYUBRR);

  setup_button();

  sei();
  char bmpbuf[20];
  char rpmbuf[20];
  char pwmbuf[20];
  char tgtbuf[20];
  char lapbuf[20];
  char recbuf[20];
  for(;;) {
    // The main control loop. Compares control timer counter values and calls
    // the control functions at suitable intervals.
    uint8_t bp = ~BUMPER_PIN;
    // compensate for finishline.

    if (get_hamming_weight(bp) > 3) {
      bp = 0b00010000;
    }

    if (check_finish_line()) {

    }
    if (lap_timer_counter > LAPTIME_LOOP_COUNT) {
      update_laptime();
      lap_timer_counter = 0;
    }
    if (str_timer_counter > STEERING_LOOP_COUNT) {
      str_timer_counter = 0;
      read_bumper_turn_wheels(bp);
    }
    
    if (rpm_timer_counter > RPM_LOOP_COUNT) {
      update_rpm();
      update_acceleration();
      rpm_timer_counter = 0;
    }
    // Button debounce timer
    if (btn_delay == 1) {
      if (btn_timer_counter > BTN_LOOP_COUNT) {
        btn_delay = 0;
        PORTC = ~PORTC;
        btn_timer_counter = 0;
      }
    }
    // Handle the button press. Toggle motor and update screen
    if (!(PINE & 1 << PE5) && btn_delay == 0) {
      toggle_motor();
      btn_delay = 1;
      btn_timer_counter = 0;
      // sprintf(rpmbuf, "RPM:     %d  ", get_rpm());
      // output_string(rpmbuf,1,3);

      // sprintf(bmpbuf, "BUMPER:  %d  ", get_bumper_float());
      // output_string(bmpbuf, 1, 4);

      // sprintf(pwmbuf, "PWM:     %d  ", get_pwm());
      // output_string(pwmbuf, 1, 5);

      // sprintf(tgtbuf, "tgt:     %d  ", get_target_rpm());
      // output_string(tgtbuf, 1, 6);

      // sprintf(lapbuf, "LAP: %d - %d.%d",
      //   get_current_lap(), get_laptime_secs(), get_laptime_partial());
      // output_string(lapbuf, 1, 7);

      // sprintf(recbuf, "RECORD: %d - %d.%d",
      //   get_lap_record_lap(), get_lap_record_secs(), get_lap_record_partial());
      // output_string(recbuf, 1, 8);

      laps = get_lap_lengths();
      sprintf(rpmbuf, "lap1:-%ld------", laps[1]);
      output_string(recbuf, 1, 1));

    }
  }
}

ISR(TIMER2_COMPA_vect) {
  // increment the control timers at homogenous rate.
  str_timer_counter++;
  rpm_timer_counter++;
  btn_timer_counter++;
  lap_timer_counter++;
  scr_timer_counter++;
  finish_line_counter++;
}
