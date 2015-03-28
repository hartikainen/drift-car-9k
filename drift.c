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
#define PREV_BP_COUNT 3
#define LEFT_TURN_MASK 0b11100000
#define STRAIGHT_MASK 0b00011000
#define RIGHT_TURN_MASK 0b00000111
#define LEFT_STEERING 0b00000100
#define STRAIGHT_STEERING 0b00000010
#define RIGHT_STEERING 0b00000001

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

uint8_t track_info[800];
void inspect_track(void) {
  static unsigned int prev_tcnt = 0;
  // prev_bp works as a queue data structure
  static uint8_t prev_bp[PREV_BP_COUNT];
  static int on_straight = 1;

  unsigned int tcnt = TCNT5;
  uint8_t bp = ~BUMPER_PIN;
  int i;

  if (tcnt > prev_tcnt) {
    prev_tcnt = tcnt;

    // Push bp to the prev_bp queue
    for (i=0; i<PREV_BP_COUNT-1; i++) {
      prev_bp[i] = prev_bp[i+1];
    }
    prev_bp[PREV_BP_COUNT] = bp;

    uint8_t old_bp;
    uint8_t same_steering = 0b00000000;

    for (i=0; i<PREV_BP_COUNT; i++) {
      old_bp = prev_bp[i];
      if (((old_bp & RIGHT_TURN_MASK) > 0) && !(old_bp & STRAIGHT_MASK)) { // if turning right
	same_steering = same_steering | 0b00000001;
      } else if (((old_bp & LEFT_TURN_MASK) > 0) && !(old_bp & STRAIGHT_MASK)) { // if turning left
	same_steering = same_steering | 0b00000100;
      } else if (((old_bp & STRAIGHT_MASK)) && !(old_bp & (LEFT_TURN_MASK | RIGHT_TURN_MASK))) { // if going straight
	same_steering = same_steering | 0b00000010;
      } 
    }

    if (on_straight) {
      if (same_steering == RIGHT_STEERING) {
	// Right turn started 3 steps ago
	on_straight = 0;
	track_info[tcnt] = same_steering;
      } else if (same_steering == LEFT_STEERING) {
	// Left turn started 3 steps ago
	on_straight = 0;
	track_info[tcnt] = same_steering;
      } else {
	track_info[tcnt] = STRAIGHT_STEERING;

      }
    } else {
      if (same_steering == STRAIGHT_STEERING) {
	// Straight started 3 steps ago
	on_straight = 1;
	track_info[tcnt] = same_steering;
      } else if (same_steering == LEFT_STEERING) {
	track_info[tcnt] = same_steering;
      } else if (same_steering == RIGHT_STEERING) {
	track_info[tcnt] = same_steering;
      }
    }
  }
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

    inspect_track();
    // The main control loop. Compares control timer counter values and calls
    // the control functions at suitable intervals.
    uint8_t bp = ~BUMPER_PIN;
    // compensate for finishline.
    if (get_hamming_weight(bp) > 3) {
      bp = 0b00010000;
    }
    if (lap_timer_counter > LAPTIME_LOOP_COUNT) {
      update_laptime();
      lap_timer_counter = 0;
    }
    if (str_timer_counter > STEERING_LOOP_COUNT) {
      str_timer_counter = 0;
      read_bumper_turn_wheels(bp);
    }
    if (finish_line_counter > FINISHLINE_LOOP_COUNT) {
      check_finish_line();
      finish_line_counter = 0;
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
      int *laps = get_laps();
      sprintf(rpmbuf, "lap1:     %d  ", laps[1]);
      output_string(rpmbuf,1,3);

      sprintf(bmpbuf, "lap2:     %d  ", laps[2]);
      output_string(bmpbuf, 1, 4);

      sprintf(pwmbuf, "lap3:     %d  ", laps[3]);
      output_string(pwmbuf, 1, 5);

      sprintf(tgtbuf, "lap4:     %d  ", laps[4]);
      output_string(tgtbuf, 1, 6);

      sprintf(lapbuf, "LAP: %d - %d.%d",
        get_current_lap(), get_laptime_secs(), get_laptime_partial());
      output_string(lapbuf, 1, 7);

      sprintf(recbuf, "RECORD: %d - %d.%d",
        get_lap_record_lap(), get_lap_record_secs(), get_lap_record_partial());
      output_string(recbuf, 1, 8);

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
