#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/delay.h>

#include "accelerate.h"
#include "output.h"
#include "bumper.h"
#include "PID.h"

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

void display_example(void)
{
  // String2 is the command to be sent to the display.
  // See the 'Draw “String” of ASCII Text (text format)'
  // -function in the display command set reference.
  char String2[] = {'s', 0x1, 0x1, 0x3, 0xFF, 0xFF, 't', 'e', 's', 't', 'i', 0x00};
  _delay_ms(500);

  // Transmit the initial 'AutoBaud' command. This is done only once.
  USART_transmit('U');

  _delay_ms(100);

  char jiiri = USART_receive();
  // If the display answers with ACK
  if (jiiri == 0x06) {
    jiiri = 0x00;
    PORTC |= _BV(PC0) | _BV(PC1);

    USART_putstring(String2);
    USART_transmit(0x00);

    jiiri = USART_receive();
    if (jiiri) {
      PORTC &= ~_BV(PC1);
    }
  }
}


static volatile char btn_delay = 0;

volatile int btn_timer_counter = 0;
int main(void)
{

  setup_leds();
  setup_tachometer();
  setup_bumper_ddr();
  setup_timer2();

  USART_init(MYUBRR);
  PORTC = 0;

  display_example();
  output_set_opaque_text();
  setup_button();

  sei();
  char jiiri[20];
  for(;;) {
   
    if (!(PINE & 1 << PE5) && btn_delay == 0) {
      toggle_motor();
      btn_delay = 1;
      btn_timer_counter = 0;
      //      reset_PID_stuff();
    }
  }
}

#define STEERING_LOOP_COUNT 1
#define RPM_LOOP_COUNT 50

volatile char str_timer_counter = 0;
volatile char rpm_timer_counter = 0;   // remove at least one counter
volatile unsigned int last_rpm = 0;

ISR(TIMER2_COMPA_vect) {
  PORTC = ~PORTC; // TEST, blink the leds
  if (str_timer_counter++ > STEERING_LOOP_COUNT) {
    str_timer_counter = 0;
    read_bumper_turn_wheels();

  }
  if (rpm_timer_counter++ > RPM_LOOP_COUNT) {
    update_acceleration(60);
    rpm_timer_counter = 0;
  }
  if (btn_delay == 1){
    if (btn_timer_counter++ > BTN_LOOP_COUNT) {
      btn_delay = 0;
      PORTC = ~PORTC;
    }
  }
}
