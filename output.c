#include <avr/io.h>
#include <avr/delay.h>
#include "output.h"

// Straight from the [datasheet, p. 211]
void USART_init(unsigned int ubrr) {
  /* Set baud rate */
  UBRR1H = (unsigned char) (ubrr>>8);
  UBRR1L = (unsigned char) ubrr;
  /* Enable receiver and transmitter */
  UCSR1B |= (1<<RXEN1) | (1<<TXEN1);
  /* Set frame format: 8 data bits, no parity, 1 stop bit */
  UCSR1C |= (1<<UCSZ10) | (1<<UCSZ11);
}

void USART_putstring(char *str_ptr) {
  while(*str_ptr != '\0') {
    USART_transmit(*str_ptr++);
  }
}

void USART_transmit(unsigned char data) {
  while( !(UCSR1A & (1 << UDRE1)) );
  UDR1 = data;
}

unsigned char USART_receive(void) {
  while( !(UCSR1A & (1 << RXC1)) ) {
    _delay_ms(100);
    PINC = (1 << PC0) | (1 << PC1);
  }
  return UDR1;
}
