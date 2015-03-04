#include <inttypes.h> /* definitions for uint8_t and others */
#include <avr/io.h>   /* definitions for all PORT* and other registers. You absolutely will need this one */
#include <avr/interrupt.h>

#include <avr/delay.h>


void setup_motor_pwm(int pwmoffset)
{
    PORTK |= 1 << PK0;
    DDRH |= 1 << PH3; // PH3 OC4A (Output Compare and PWM Output A for Timer/Counter4)
    TCCR4A |= 1 << WGM41 | 1 << COM4A1 | 1 << COM4A0;
    TCCR4B |= 1 << WGM43 | 1 << WGM42 | 1 << CS40;
    ICR4 = 800;
    OCR4A = ICR4 - pwmoffset;
}

void disable_motor_pwm(void)
{
   TCCR4A |= ~_BV(COM4A1);
}


int main(void) {
    setup_motor_pwm(140);
    for(;;) {
    }
}
