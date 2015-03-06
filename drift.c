#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/delay.h>

#include "output.h"
#include "bumper.h"

void setup_motor_pwm(int pwmoffset) {
  PORTK |= 1 << PK0;
  DDRH |= 1 << PH3; // PH3 OC4A (Output Compare and PWM Output A for Timer/Counter4)
  TCCR4A |= 1 << WGM41 | 1 << COM4A1 | 1 << COM4A0;
  TCCR4B |= 1 << WGM43 | 1 << WGM42 | 1 << CS40;
  ICR4 = 800; // 20kHz without prescaler
  OCR4A = ICR4 - pwmoffset;
}

void disable_motor_pwm(void) {
  TCCR4A |= ~_BV(COM4A1);
}

void setup_leds(void) {
  DDRC = 0xff;
}

void setup_tachometer(void) {
  DDRL &= ~(1<<PL2);
  TCCR5B |= 1 << CS51 | 1 << CS52;
  OCR5A = 10;
}

void setup_pwm(int val) {
  DDRB = 0xff;
  TCCR1A |= (1 << COM1A0) | (1 << COM1A1) | (1 << WGM11);
  TCCR1B |= (1 << CS10) | (1 << CS11) | (1 << WGM12) | (1 << WGM13);
  ICR1 = 4999;
  OCR1A = ICR1 - val;
}

void display_example(void)
{
  // String2 is the command to be sent to the display.
  // See the 'Draw “String” of ASCII Text (text format)'
  // -function in the display command set reference.
  char String2[] = {'s', 0x1, 0x1, 0x3, 0xFF, 0xFF, 't', 'e', 's', 't', 'i', 0x00};

  _delay_ms(2000);
  // Transmit the initial 'AutoBaud' command. This is done only once.
  USART_transmit('U');

  _delay_ms(2000);

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

int main(void)
{
  //  setup_motor_pwm(140);
  setup_leds();
  setup_tachometer();
  setup_bumper_ddr();
  setup_bumper_timer();

  USART_init(MYUBRR);
  PORTC = 0;

  output_set_opaque_text();
  display_example();

  sei();

  for(;;) {
    read_bumper_turn_wheels();
  }
}

int LOOP_COUNT = 10;
volatile int timer_counter = 0;
ISR(TIMER2_COMPA_vect) {
  if (timer_counter++ > LOOP_COUNT) {
    timer_counter = 0;
    release_steering();
  }
}


ISR(TIMER5_CAPT_vect) {
  PINC |= _BV(PC0);
}

/* THE DISPLAY STUFF,
 * THESE SHOULD PROBABLY
 * BE MOVED TO ANOTHER FILE
 */
