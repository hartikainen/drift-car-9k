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

void setup_button()
{
  DDRE = 0x0; // 0 or FF?
}

void setup_tachometer(void) {
  DDRL &= ~(1<<PL2);
  TCCR5B |= 1 << CS51 | 1 << CS52;
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

  display_example();
  output_set_opaque_text();


  sei();
  char jiiri[20];
  for(;;) {
    if ((PINE == 0b00110011) ){//== 0b00010000) {
      output_string(sprintf("asdasdo: %s", itoa(PINE, jiiri, 2)));
      //      output_string("toimii");
    } else {
      output_string(sprintf("jiiri2: %s !", itoa(PINE, jiiri, 2)));
    }

    read_bumper_turn_wheels();
  }
}

#define STEERING_LOOP_COUNT 10
#define RPM_LOOP_COUNT 50

volatile char str_timer_counter = 0;
volatile char rpm_timer_counter = 0;   // remove at least one counter
volatile unsigned int last_rpm = 0;

ISR(TIMER2_COMPA_vect) {
  if (str_timer_counter++ > STEERING_LOOP_COUNT) {
    str_timer_counter = 0;
    release_steering();
  }
  if (rpm_timer_counter++ > RPM_LOOP_COUNT) {
    rpm_timer_counter = 0;
    char rpmstr[10];
    itoa((TCNT5-last_rpm), rpmstr, 10);    // weird rpm when TCNT5 overflows
    last_rpm = TCNT5;

    output_string(rpmstr);
  }
}


ISR(TIMER5_CAPT_vect) {
  PINC |= _BV(PC0);
}