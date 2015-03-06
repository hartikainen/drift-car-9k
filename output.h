#define BAUD 9600
#define FOSC 1843200 // f_{OSC}, e.g. the clock speed, this might be available in F_CPU?
#define MYUBRR (F_CPU/(16UL*BAUD) - 1)

void USART_init(unsigned int ubrr);
void USART_putstring(char *str_ptr);
void USART_transmit(unsigned char data);
unsigned char USART_receive(void);

void output_clear(void);
void output_string_append(void);

