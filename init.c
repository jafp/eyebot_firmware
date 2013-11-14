/*--------------------------------------------------------

Init.c



Author: 	Andre Daniel B Christensen
Project:        Eyebot 4.semester DTU
Date:		2013-09-20

--------------------------------------------------------*/
#define F_CPU 16000000
#define ADDR 0x4E
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>


#include <avr/interrupt.h>
#include <compat/twi.h>
#include <avr/interrupt.h>

#include "init.h"
#include "motor.h"


void int_init(void){
        DDRA |= (1<<PA0);
        DDRC |= (1<<PC0);

   // Prescaler = FCPU/1024
   TCCR1B|=(1<<CS10);

   //Enable Overflow Interrupt Enable
   TIMSK|=(1<<TOIE1);

   //Initialize Counter
   TCNT1L=0;

   //Initialize our varriable
   MCUCR = 15;

   GIMSK |= (1<<INTF1) | (1<<INTF0);

   //Port C[3,2,1,0] as out put
   DDRA = 0b11111111;;
   DDRC |= (1<<PC0);

   //Enable Global Interrupts
 

}

