#include <inttypes.h> /* definitions for uint8_t and others */
#include <avr/io.h>   /* definitions for all PORT* and other registers. You absolutely will need this one */
#include <avr/interrupt.h>

#include <avr/delay.h>

#define BUMPER_PORT PORTA
#define BUMPER_DDR DDRA
#define BUMPER_PIN PINA

// DEFINITIONS FOR THE DISPLAY
#define BAUD 9600
#define FOSC 1843200 // f_{OSC}, e.g. the clock speed, this might be available in F_CPU?
#define MYUBRR (FOSC/16/BAUD-1)

<<<<<<< HEAD
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
  TCCR5B |= 1 << CS50 | 1 << ICES5;
  TIFR5 = 1<< ICF5;
  TIMSK5 |= (1 << ICIE5);
}

void setup_ddr(void) {
  BUMPER_DDR = 0;
}

// Straight from the [datasheet, p. 211]
void USART_Init(unsigned int ubrr) {
  /* Set baud rate */
  UBRR0H = (unsigned char) (ubrr>>8);
  UBRR0L = (unsigned char) ubrr;
  /* Enable receiver and transmitter */
  UCSR1B = (1<<RXEN1) | (1<<TXEN1);
  /* Set frame format: 8data, 2stop bit */
  UCSR1C = (1<<USBS1) | (3<<UCSZ10);
}

void USART_putstring(char* StringPtr) {
  while(*StringPtr != 0x00) {
    USART_send(*StringPtr);
    StringPtr++;
  }
}

void USART_send(unsigned char data) {
  while( !(UCSR1A & (1 << UDRE1)) );
  UDR0 = data;
}

unsigned char USART_receive(void) {
  while(!(UCSR1A & (1<<RXC1)));
  return UDR0;
}

void reset_timer(void) {
  TCNT2 = 0;
}

void setup_bumper_wheel_timer(void) {
  TCCR2B |= (1 << WGM22) | (1 << CS22) | (1 << CS21) | (1 << CS20);
  TIMSK2 |= (1 << OCIE2A);
  OCR2A = 0b11111111;
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
int main(void)
{
  uint8_t bumper;

  setup_motor_pwm(140);
  setup_leds();
  setup_tachometer();

  char String[] = "JIIRI";
  int testi = 1;
  char test2;

  setup_ddr();
  
  //  sbi(DDRB, 0);
  //  cbi(PORTB, 0);
  USART_Init(MYUBRR);
  
  setup_bumper_wheel_timer();
  sei();
  for(;;) {
    //    if (testi) USART_send("Z");
    //    test2 = USART_receive();
    if (test2 == 0x06) {testi = 0;}
    
    bumper = ~BUMPER_PIN;
    if (!steering_locked) {
      steering_locked = 1;
      reset_timer();

      if (bumper > 0) {
	USART_send("s");
	//	USART_putstring(String);
      }

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

int LOOP_COUNT = 10;
volatile int timer_counter = 0;
ISR(TIMER2_COMPA_vect) {
  if (timer_counter++ > LOOP_COUNT) {
    timer_counter = 0;
    steering_locked = 0;
    TCCR1A &= ~_BV(COM1A1);
  }
}


ISR(TIMER5_CAPT_vect) {
  PINC |= _BV(PC0);
}

/* THE DISPLAY STUFF,
 * THESE SHOULD PROBABLY
 * BE MOVED TO ANOTHER FILE
 */
