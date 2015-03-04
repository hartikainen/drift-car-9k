#include <inttypes.h> /* definitions for uint8_t and others */
#include <avr/io.h>   /* definitions for all PORT* and other registers. You absolutely will need this one */
#include <avr/interrupt.h>

#include <delay.h>


void setup_pwm(int pwmoffset)
{
    DDRH |= 1 << PH3; // PH3 OC4A (Output Compare and PWM Output A for Timer/Counter4)
    TCCR4A |= 1 << WGM41 | 1 << COM4A1 | 1 << COM4A0;
    TCCR4B |= 1 << WGM43 | 1 << WGM42 | 1 << CS40 | 1 << CS41;
    ICR1 = 4999;
    OCR1A = ICR1 - pwmoffset;
}

void disable_pwm(void)
{
    TCCR1A &= ~_BV(COM1A1);
}


int main(void) {
    setup_pwm(500);
    _delay_ms(500);
    disable_pwm;
    for(;;) {
    }
}
