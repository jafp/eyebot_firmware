

#ifndef _MOTOR_H_
#define _MOTOR_H_

#include <stdint.h>

void motor_init(void);
void motor_set(uint8_t speed_left, uint8_t speed_right, uint8_t direct_l, 
	uint8_t direct_r );

#endif

