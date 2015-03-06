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
}

void output_clear(void)
{
  char msg  = 'E';
  USART_transmit(msg);
}

void output_string(char* string)
{
  int length = 6 + strlen(string) + 1; // 6 for the initial command, 1 for the terminate char
  char msg[50] = {'s', 0x1, 0x1, 0x3, 0xFF, 0xFF};
  strcat(msg, string);
  strcat(msg, 0x00); // concat the terminate char

  USART_putstring(msg);
  USART_transmit(0x00);
}