#include <avr/io.h>
#include <avr/delay.h>
#include "output.h"
#include <string.h>
// Straight from the [datasheet, p. 211]
void USART_init(unsigned int ubrr)
{
  /* Set baud rate */
  UBRR1H = (unsigned char) (ubrr>>8);
  UBRR1L = (unsigned char) ubrr;
  /* Enable receiver and transmitter */
  UCSR1B |= (1<<RXEN1) | (1<<TXEN1);
  /* Set frame format: 8 data bits, no parity, 1 stop bit */
  UCSR1C |= (1<<UCSZ10) | (1<<UCSZ11);

  send_autobaud();
}

void USART_putstring(char *str_ptr)
{
  while(*str_ptr != '\0') {
    USART_transmit(*str_ptr++);
  }
}

void USART_transmit(unsigned char data)
{
  while( !(UCSR1A & (1 << UDRE1)) );
  UDR1 = data;
}

unsigned char USART_receive(void)
{
  while( !(UCSR1A & (1 << RXC1)) );
  return UDR1;
}

void output_set_opaque_text(void)
{
  USART_transmit(0x4F);
  USART_transmit(0x01);
  USART_receive();
}

void output_clear(void)
{
  char msg  = 'E';
  USART_transmit(msg);
}

void output_string(char* string, char x, char y)
{
  int length = 6 + strlen(string) + 1; // 6 for the initial command, 1 for the terminate char
  char output[50] = {'s', x, y, 0x3, 0xFF, 0xFF};
  strcat(output, string);
  strcat(output, 0x00); // concat the terminate char

  USART_putstring(output);
  USART_transmit(0x00);
  USART_receive(); // wait for ACK
}

void send_autobaud(void)
{
  PORTC |= _BV(PC0) | _BV(PC1);
  _delay_ms(500); // Waiting for the screen to wake up
  // Transmit the initial 'AutoBaud' command. This is done only once.
  USART_transmit('U');

  // If the display answers with ACK
  if (USART_receive() == 0x06) {
    PORTC |= _BV(PC0) | _BV(PC1);
    //PORTC &= ~_BV(PC1);

    output_set_opaque_text();
    output_string("Screen initialized", 1, 1);
    output_string("(.) (.)", 6, 18);

  }
}

