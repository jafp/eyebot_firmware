/*--------------------------------------------------------

Init.h



Author: 	Andre Daniel B Christensen
Project:        Eyebot 4.semester DTU
Date:		2013-09-20

--------------------------------------------------------*/

#ifndef _Init_h_
#define _Init_h_

#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <compat/twi.h>
#include <avr/interrupt.h>

void int_init(void);
void init_i2c(void);

#endif

