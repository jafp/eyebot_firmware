
#include <avr/io.h>

#include "pwm.h"

void pwm_init(void)  // initialize timer in PWM mode
{
    // initialize TCCR0 and TCCR2 to requiret setting with no prescalling and /WGM bit 0 and bit 1 is PWM fast mode 
    // COM bit 1 is  Clear OC2 on compare match, set OC2 at BOTTOM,(non-inverting mode)
    // CS bit is the clock prescaling.
    TCCR0 |= (1<<WGM00)|(1<<WGM01)|(1<<COM01)|(1<<CS01);
    TCCR2 |= (1<<WGM20)|(1<<WGM21)|(1<<COM21)|(1<<CS21);
 
    // setting pin PB 0,1,2,3,4 and PD 7 as output
   
    DDRB |= (1<<PB4)|(1<<PB3)|(1<<PB2)|(1<<PB1)|(1<<PB0);
    DDRD |= (1<<PD7);

   //Enable the time Overflow Interrupt 
   //TIMSK|=(1<<TOIE0) | (1<<TOIE2);

   //Initialize Counter to zero for timer 0 and 2.
   TCNT0 = 0;	
   TCNT2 = 0;
}
