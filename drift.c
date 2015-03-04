#include <inttypes.h> /* definitions for uint8_t and others */
#include <avr/io.h>   /* definitions for all PORT* and other registers. You absolutely will need this one */
#include <avr/interrupt.h>

// TODO: remove this
#include <util/delay.h>

#define BUMPER_PORT PORTA
#define BUMPER_DDR DDRA
#define BUMPER_PIN PINA

#define LEDS_PORT PORTC
#define LEDS_DDR  DDRC
#define LEDS_PIN  PINC

#define BLINK_LED PC2
#define OTHER_LED PC0

void setup_ddr(void) {
  BUMPER_DDR = 0;
}

void reset_timer(void) {
  TCNT2 = 0;
}

void setup_bumper_wheel_timer(void) {
  TCCR2B |= (1 << WGM22) | (1 << CS20) | (1 << CS21);
  TIMSK2 |= (1 << OCIE2A);
  OCR2A = 254;
}

void setup_pwm(int val) {
  DDRB = 0xff;
  TCCR1A |= (1 << COM1A0) | (1 << COM1A1) | (1 << WGM11);
  TCCR1B |= (1 << CS10) | (1 << CS11) | (1 << WGM12) | (1 << WGM13);
  ICR1 = 4999;
  OCR1A = ICR1 - val;
}

int MIDDLE = 370;
volatile int steering_locked = 0;
int main(void) {
  uint8_t bumper;

  setup_ddr();
  setup_bumper_wheel_timer();
  sei();

  for(;;) {
    bumper = ~BUMPER_PIN;
    
    if (!steering_locked) {
      steering_locked = 1;
      reset_timer();

      switch (bumper) {
      case 0b00000001:
	setup_pwm(MIDDLE - 40);
	break;
      case 0b00000010:
	setup_pwm(MIDDLE - 30);
	break;
      case 0b00000100:
	setup_pwm(MIDDLE - 20);
	break;
      case 0b00001000:
	setup_pwm(MIDDLE - 10);
	break;
      case 0b00010000:
	setup_pwm(MIDDLE + 10);
	break;
      case 0b00100000:
	setup_pwm(MIDDLE + 20);
	break;
      case 0b01000000:
	setup_pwm(MIDDLE + 30);
	break;
      case 0b10000000:
	setup_pwm(MIDDLE + 40);
	break;
      default:
	setup_pwm(MIDDLE);
      }
    }
  }
}

ISR(TIMER2_COMPA_vect) {
  steering_locked = 0;
  TCCR1A &= ~_BV(COM1A1);
}
